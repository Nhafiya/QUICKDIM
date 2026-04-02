#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include "Adafruit_VL53L0X.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// ===========================
// Configuration
// ===========================
const char* ssid     = "GalaxyS23";
const char* password = "12345678";

// I2C pins for ESP32-CAM
// GPIO 12 is a STRAPPING PIN (flash voltage) — NEVER use for I2C!
// GPIO 13 = SDA, GPIO 14 = SCL are safe choices
#define I2C_SDA 13
#define I2C_SCL 14

// Buzzer & Button pins (safe GPIOs on ESP32-CAM)
#define BUZZER_PIN  2   // Active/passive buzzer, connect directly to GPIO 2 + GND
#define BUTTON_PIN 15   // Push button between GPIO 15 and GND (internal pull-up)
#define BUZZER_FREQ 2700 // Hz — typical piezo resonant frequency (loudest)

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT  64
#define OLED_RESET     -1
#define OLED_ADDRESS 0x3C   // Change to 0x3D if display not found

// ===========================
// AI Thinker Camera Pins
// ===========================
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// ===========================
// Globals
// ===========================
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

WebServer server(80);
WiFiServer streamServer(81);

bool hasDisplay = false;
bool hasToF     = false;

// Buzzer & Button state
bool measureLocked = false;       // When true, measurement display is frozen
unsigned long lastDebounce = 0;
const unsigned long DEBOUNCE_MS = 250;

// Last measurement (for lock display)
String lastLabel = "";
String lastSize  = "";
String lastDist  = "";

// Non-blocking buzzer
bool buzzerActive = false;
unsigned long buzzerStart = 0;
int buzzerBeepsLeft = 0;
int buzzerDuration = 100;         // ms per beep
bool buzzerOnPhase = true;

// ===========================
// Stream Constants
// ===========================
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* STREAM_CONTENT_TYPE =
  "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* STREAM_BOUNDARY =
  "\r\n--" PART_BOUNDARY "\r\n";
static const char* STREAM_PART =
  "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

// ===========================
// I2C Scanner (debug helper)
// ===========================
void scanI2C() {
  Serial.println("Scanning I2C bus...");
  bool found = false;
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  I2C device found at 0x%02X\n", addr);
      found = true;
    }
  }
  if (!found) Serial.println("  No I2C devices found!");
}

// ===========================
// Display Helper
// ===========================
void updateDisplay(const String& title,
                   const String& line1,
                   const String& line2,
                   const String& line3) {
  if (!hasDisplay) return;
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(title);
  display.println("-------------");
  if (line1.length() > 0) display.println(line1);
  if (line2.length() > 0) display.println(line2);
  if (line3.length() > 0) display.println(line3);
  display.display();
}

// ===========================
// Buzzer Helper (non-blocking)
// ===========================
void startBuzz(int count = 2, int duration_ms = 100) {
  buzzerBeepsLeft = count;
  buzzerDuration = duration_ms;
  buzzerActive = true;
  buzzerOnPhase = true;
  buzzerStart = millis();
  ledcWriteTone(BUZZER_PIN, BUZZER_FREQ);  // PWM tone — LOUD
}

void updateBuzzer() {
  if (!buzzerActive) return;
  unsigned long elapsed = millis() - buzzerStart;
  if (elapsed >= (unsigned long)buzzerDuration) {
    if (buzzerOnPhase) {
      // End of ON phase -> silence
      ledcWriteTone(BUZZER_PIN, 0);
      buzzerOnPhase = false;
      buzzerStart = millis();
    } else {
      // End of OFF phase -> next beep or stop
      buzzerBeepsLeft--;
      if (buzzerBeepsLeft <= 0) {
        buzzerActive = false;
        return;
      }
      ledcWriteTone(BUZZER_PIN, BUZZER_FREQ);
      buzzerOnPhase = true;
      buzzerStart = millis();
    }
  }
}

// ===========================
// Button Handler (debounced)
// ===========================
void checkButton() {
  if (digitalRead(BUTTON_PIN) == LOW) {  // Pressed (pulled to GND)
    if (millis() - lastDebounce > DEBOUNCE_MS) {
      lastDebounce = millis();
      measureLocked = !measureLocked;
      Serial.println(measureLocked ? "[BTN] Locked" : "[BTN] Unlocked");
      startBuzz(1, 50);  // Short confirmation beep
      if (hasDisplay) {
        if (measureLocked && lastLabel.length() > 0) {
          // Show last measurement with LOCKED header
          updateDisplay("== LOCKED ==",
                        "Obj: " + lastLabel,
                        lastSize,
                        lastDist);
        } else if (measureLocked) {
          updateDisplay("== LOCKED ==", "No measurement yet", "Press btn to unlock", "");
        }
        // On unlock, display will refresh on next measurement
      }
    }
  }
}

// ===========================
// HTTP Handlers
// ===========================
void handleToF() {
  int distance = 0;
  if (hasToF) {
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);
    distance = (measure.RangeStatus != 4) ? measure.RangeMilliMeter : 0;
  } else {
    distance = -1;
  }
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json",
              "{\"dist\":" + String(distance) + "}");
}

void handleDisplay() {
  String w    = server.arg("w");
  String h    = server.arg("h");
  String dist = server.arg("dist");
  String label = server.arg("label");

  // Always store last measurement
  lastLabel = label;
  lastSize  = "Size: " + w + "x" + h + " mm";
  lastDist  = "Dist: " + dist + " mm";

  // Only update OLED if not locked
  if (!measureLocked) {
    updateDisplay("Measurement",
                  "Obj: " + label,
                  lastSize,
                  lastDist);
  }

  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "text/plain", "OK");
}

void handleBuzz() {
  int count = server.hasArg("count") ? server.arg("count").toInt() : 2;
  int ms    = server.hasArg("ms")    ? server.arg("ms").toInt()    : 100;
  if (count < 1) count = 1;
  if (count > 5) count = 5;
  if (ms < 30) ms = 30;
  if (ms > 500) ms = 500;
  startBuzz(count, ms);
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", "{\"buzzing\":true}");
}

void handleButton() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json",
              "{\"locked\":" + String(measureLocked ? "true" : "false") + "}");
}

void handleRoot() {
  String ip = WiFi.localIP().toString();
  String html = R"rawhtml(
<!DOCTYPE html><html><head>
<title>ESP32-CAM Measure</title>
<style>
  body { font-family: monospace; background:#111; color:#0f0;
         display:flex; flex-direction:column; align-items:center; padding:20px; }
  img  { border:2px solid #0f0; max-width:640px; width:100%; }
  .info { margin-top:12px; font-size:1.2em; }
</style></head><body>
<h2>ESP32-CAM Live Stream</h2>
<img src="http://)rawhtml" + ip + R"rawhtml(:81" /><br>
<div class="info">
  Stream: <a href="http://)rawhtml" + ip + R"rawhtml(:81" style="color:#0f0">http://)rawhtml" + ip + R"rawhtml(:81</a><br>
  ToF:    <a href="http://)rawhtml" + ip + R"rawhtml(/tof"  style="color:#0f0">http://)rawhtml" + ip + R"rawhtml(/tof</a>
</div>
</body></html>
)rawhtml";
  server.send(200, "text/html", html);
}

// ===========================
// Stream Task (Core 1)
// ===========================
void streamTask(void* pvParameters) {
  Serial.println("[STREAM] Task running, waiting for clients...");
  while (true) {
    WiFiClient client = streamServer.available();
    if (client) {
      Serial.println("[STREAM] Client connected");
      client.setTimeout(3);
      client.printf(
        "HTTP/1.1 200 OK\r\nContent-Type: %s\r\n"
        "Access-Control-Allow-Origin: *\r\n\r\n",
        STREAM_CONTENT_TYPE);

      while (client.connected()) {
        camera_fb_t* fb = esp_camera_fb_get();
        if (!fb) { delay(10); continue; }

        char part_buf[128];
        size_t part_len = sprintf(part_buf, STREAM_PART, fb->len);

        size_t written = 0;
        written += client.write((uint8_t*)STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
        written += client.write((uint8_t*)part_buf, part_len);
        written += client.write(fb->buf, fb->len);

        esp_camera_fb_return(fb);

        if (written == 0) {
          Serial.println("[STREAM] Write failed, dropping client");
          break;
        }
      }
      client.stop();
      Serial.println("[STREAM] Client disconnected");
    }
    delay(1);
  }
}

// ===========================
// Camera Init
// ===========================
bool initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
    config.frame_size   = FRAMESIZE_VGA;
    config.jpeg_quality = 10;
    config.fb_count     = 2;
  } else {
    config.frame_size   = FRAMESIZE_VGA;
    config.jpeg_quality = 12;
    config.fb_count     = 1;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    return false;
  }

  // Optional: improve image quality
  sensor_t* s = esp_camera_sensor_get();
  if (s) {
    s->set_brightness(s, 0);
    s->set_contrast(s, 0);
    s->set_saturation(s, 0);
    s->set_whitebal(s, 1);
    s->set_awb_gain(s, 1);
    s->set_exposure_ctrl(s, 1);
    s->set_aec2(s, 0);
  }

  return true;
}

// ===========================
// Setup
// ===========================
void setup() {
  // Disable brownout FIRST
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(115200);
  delay(500);   // Let power rails stabilize before touching I2C/camera
  Serial.println("\n--- ESP32-CAM YOLO Auto-Measure ---");

  // ---- 0. Buzzer & Button ----
  ledcAttach(BUZZER_PIN, BUZZER_FREQ, 8);  // Attach pin, 2700Hz, 8-bit
  ledcWriteTone(BUZZER_PIN, 0);            // Start silent
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  Serial.println("Buzzer: GPIO 2 (PWM), Button: GPIO 15");

  // ---- 1. Camera first (uses its own SCCB on GPIO 26/27) ----
  Serial.println("Initializing camera...");
  if (!initCamera()) {
    Serial.println("Camera init FAILED — halting.");
    while (true) delay(1000);
  }
  Serial.println("Camera OK");

  // ---- 2. I2C on safe pins (13/12), AFTER camera ----
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);   // 100 kHz — more reliable for long wires
  delay(100);
  scanI2C();               // Prints found addresses — check Serial Monitor

  // ---- 3. OLED ----
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println("SSD1306 not found at 0x3C — trying 0x3D...");
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3D)) {
      Serial.println("OLED init failed on both addresses.");
      hasDisplay = false;
    } else {
      hasDisplay = true;
      Serial.println("OLED found at 0x3D");
    }
  } else {
    hasDisplay = true;
    Serial.println("OLED found at 0x3C");
  }

  if (hasDisplay) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Booting...");
    display.display();
  }

  // ---- 4. VL53L0X ToF ----
  if (!lox.begin()) {
    Serial.println("VL53L0X not found");
    hasToF = false;
    updateDisplay("Sensor Error", "VL53L0X missing", "Check wiring", "");
  } else {
    hasToF = true;
    Serial.println("VL53L0X OK");
    updateDisplay("Sensors OK", "Camera: OK", "ToF: OK", "OLED: OK");
  }

  // ---- 5. WiFi ----
  WiFi.begin(ssid, password);
  updateDisplay("Connecting...", "SSID:", String(ssid), "");
  Serial.print("Connecting to WiFi");

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    String ip = WiFi.localIP().toString();
    Serial.println("WiFi connected: " + ip);
    updateDisplay("Ready!", "Stream :81", "ToF: /tof", ip);
  } else {
    Serial.println("WiFi FAILED");
    updateDisplay("WiFi Error", "Check SSID/pass", "", "");
  }

  // ---- 6. HTTP routes ----
  server.on("/",        HTTP_GET, handleRoot);
  server.on("/tof",     HTTP_GET, handleToF);
  server.on("/display", HTTP_GET, handleDisplay);
  server.on("/buzz",    HTTP_GET, handleBuzz);
  server.on("/button",  HTTP_GET, handleButton);
  server.begin();
  Serial.println("HTTP server started on port 80");

  // ---- 7. Start stream server HERE (on Core 0 with WiFi) ----
  streamServer.begin();
  streamServer.setNoDelay(true);
  Serial.println("Stream server listening on port 81");

  // ---- 8. Stream task on Core 0 (same as WiFi) ----
  xTaskCreatePinnedToCore(
    streamTask, "StreamTask",
    10240, NULL, 1, NULL, 0);
  Serial.println("Stream task started");
}

// ===========================
// Loop
// ===========================
void loop() {
  server.handleClient();
  checkButton();
  updateBuzzer();
  delay(10);
}
