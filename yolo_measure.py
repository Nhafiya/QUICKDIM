"""
ESP32-CAM YOLO Segmentation Auto-Measure (PC Processing)
========================================================
Connects to the ESP32-CAM MJPEG stream, runs YOLO segmentation,
fetches ToF distance, and calculates real-world dimensions on the PC.

Usage:
  pip install ultralytics opencv-python requests
  python yolo_measure.py

Controls:
  - Click on a detected object to measure it
  - Drag a rectangle to manually measure an area
  - Press 'c' to calibrate (click on an A4 sheet after pressing 'c')
  - Press 'r' to refresh/reconnect the stream
  - Press 'q' to quit
"""

import cv2
import numpy as np
import requests
import sys
import json
import os
import urllib.parse
import threading
import time
from urllib.request import urlopen
from ultralytics import YOLO

# ─── Configuration ───────────────────────────────────────────────
ESP32_IP = "10.76.239.188"  # ← Change this to your ESP32's IP
STREAM_URL = f"http://{ESP32_IP}:81"
TOF_URL = f"http://{ESP32_IP}/tof"
DISPLAY_URL = f"http://{ESP32_IP}/display"
BUZZ_URL = f"http://{ESP32_IP}/buzz"
BUTTON_URL = f"http://{ESP32_IP}/button"

MODEL_NAME = "yolo26n-seg.pt"
CAM_W, CAM_H = 640, 480  # Must match ESP32 FRAMESIZE_VGA

# Calibration Reference: A4 Sheet
A4_W_MM = 53.98
A4_H_MM = 85.60

# ─── Stream Reader ───────────────────────────────────────────────
class MJPEGStream:
    def __init__(self, url):
        self.url = url
        self.frame = None
        self.running = True
        self.thread = threading.Thread(target=self._update, daemon=True)
        self.thread.start()

    def _update(self):
        bytes_data = b''
        while self.running:
            try:
                stream = urlopen(self.url, timeout=10)
                while self.running:
                    chunk = stream.read(1024)
                    if not chunk: break
                    bytes_data += chunk
                    a = bytes_data.find(b'\xff\xd8')
                    if a != -1:
                        b = bytes_data.find(b'\xff\xd9', a + 2)
                        if b != -1:
                            jpg = bytes_data[a:b+2]
                            bytes_data = bytes_data[b+2:]
                            if len(jpg) > 10:
                                img = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                                if img is not None:
                                    self.frame = img
                        elif len(bytes_data) > 1024 * 1024:
                            # Safely discard to prevent memory leak
                            bytes_data = bytes_data[a:]
                    else:
                        bytes_data = bytes_data[-2:] if len(bytes_data) > 0 else b''
            except Exception as e:
                print(f"[STREAM ERR] {e}, reconnecting...")
                time.sleep(1)

    def read(self):
        if self.frame is not None:
            return True, self.frame.copy()
        return False, None

    def release(self):
        self.running = False


# ─── State ───────────────────────────────────────────────────────
calib_file = "calibration.json"
focal_px = 0.0

def load_calibration():
    global focal_px
    if os.path.exists(calib_file):
        try:
            with open(calib_file, 'r') as f:
                data = json.load(f)
                focal_px = data.get("focal_px", 0.0)
                if focal_px > 0:
                    print(f"[CAL] Loaded focal_px = {focal_px:.1f}")
        except:
            pass

def save_calibration(f):
    global focal_px
    focal_px = f
    with open(calib_file, 'w') as f_out:
        json.dump({"focal_px": f}, f_out)
    print(f"[CAL] Saved focal_px = {focal_px:.1f}")

def get_tof(callback):
    def fetch():
        try:
            r = requests.get(TOF_URL, timeout=3)
            data = r.json()
            callback(data.get("dist", 0))
        except Exception as e:
            print(f"[TOF HTTP ERR] {e}")
            callback(0)
    threading.Thread(target=fetch, daemon=True).start()

def send_display(w, h, dist, label):
    try:
        safe_label = urllib.parse.quote(label)
        url = f"{DISPLAY_URL}?w={w:.1f}&h={h:.1f}&dist={dist}&label={safe_label}"
        requests.get(url, timeout=1)
    except Exception as e:
        print(f"[DISPLAY ERR] {e}")

def send_buzz(count=2, ms=100):
    """Trigger buzzer beep on ESP32."""
    def _buzz():
        try:
            requests.get(f"{BUZZ_URL}?count={count}&ms={ms}", timeout=1)
        except:
            pass
    threading.Thread(target=_buzz, daemon=True).start()

def check_esp32():
    """Quick connectivity check before starting stream."""
    print(f"\nChecking ESP32 at {ESP32_IP}...")
    try:
        r = requests.get(TOF_URL, timeout=5)
        data = r.json()
        print(f"  ✓ ESP32 online — ToF reading: {data.get('dist', '?')}mm")
        return True
    except requests.exceptions.ConnectionError:
        print(f"  ✗ Cannot connect to {ESP32_IP}")
        print(f"    → Check ESP32 Serial Monitor for its actual IP")
        print(f"    → Make sure PC and ESP32 are on the same WiFi network")
        return False
    except requests.exceptions.Timeout:
        print(f"  ✗ Connection timed out")
        print(f"    → ESP32 may still be booting, or IP is wrong")
        return False
    except Exception as e:
        print(f"  ✗ Error: {e}")
        return False

def main():
    load_calibration()

    # Check connectivity first
    if not check_esp32():
        print("\nWould you like to continue anyway? (y/n): ", end="")
        if input().strip().lower() != 'y':
            print("Exiting.")
            return
    
    print(f"\nLoading YOLO model: {MODEL_NAME}")
    model = YOLO(MODEL_NAME)

    print(f"Connecting to stream: {STREAM_URL}")
    cap = MJPEGStream(STREAM_URL)
    
    time.sleep(2) # Wait for first frame
    if not cap.read()[0]:
        print(f"WARNING: No initial frame yet — window will show 'Connecting...'")
        
    calibrate_mode = False
    last_dims = ""
    latest_frame = None
    detections = []
    
    # Thread-safe lock for YOLO results
    det_lock = threading.Lock()

    # ─── Auto-measure tracking ───────────────────────────
    HOLD_TIME = 2.0           # seconds before auto-measure triggers
    COOLDOWN_TIME = 5.0       # seconds before re-measuring same object
    # label -> {"first_seen": time, "bbox": current, "best_bbox": best, "best_conf": float}
    tracked_objects = {}
    auto_measured = {}        # label -> last_measured_time (cooldown tracker)
    auto_measuring = False    # flag to prevent overlapping auto-measures
    measure_locked = False    # True = freeze measurement, skip auto-measure

    # Background button polling
    def button_poller():
        nonlocal measure_locked
        while running:
            try:
                r = requests.get(BUTTON_URL, timeout=1)
                data = r.json()
                new_lock = data.get("locked", False)
                if new_lock != measure_locked:
                    measure_locked = new_lock
                    print(f"[BTN] {'LOCKED' if measure_locked else 'UNLOCKED'}")
            except:
                pass
            time.sleep(0.2)

    def iou(boxA, boxB):
        """Intersection over Union for two (x0,y0,x1,y1) boxes."""
        xA = max(boxA[0], boxB[0])
        yA = max(boxA[1], boxB[1])
        xB = min(boxA[2], boxB[2])
        yB = min(boxA[3], boxB[3])
        inter = max(0, xB - xA) * max(0, yB - yA)
        if inter == 0:
            return 0.0
        areaA = (boxA[2]-boxA[0]) * (boxA[3]-boxA[1])
        areaB = (boxB[2]-boxB[0]) * (boxB[3]-boxB[1])
        return inter / (areaA + areaB - inter)

    if focal_px > 0:
        last_dims = f"Calibrated (focal={focal_px:.1f})"
    else:
        last_dims = "Not calibrated. Press 'c' then click an A4 sheet."
        
    np.random.seed(42)
    colors = np.random.randint(0, 255, (100, 3), dtype=np.uint8)

    print("\n" + "=" * 50)
    print("  PC-Side YOLO Dimension Measure")
    print("=" * 50)
    print("  Hold object 2s → auto-measure (best detection)")
    print("  Click on object → instant measure")
    print("  Drag a box → manual measure")
    print("  Press 'c' → calibrate mode (click A4 sheet)")
    print("  Press 'r' → refresh/reconnect stream")
    print("  Press 'q' → quit")
    print("=" * 50 + "\n")

    drawing = False
    start_pos = (0, 0)
    cur_pos = (0, 0)

    def on_mouse(event, mx, my, flags, param):
        nonlocal calibrate_mode, last_dims, drawing, start_pos, cur_pos
        
        h, w = param["frame_shape"][:2]
        cx, cy = int(mx * CAM_W / w), int(my * CAM_H / h)

        if event == cv2.EVENT_LBUTTONDOWN:
            drawing = True
            start_pos = (cx, cy)
            cur_pos = (cx, cy)
            
        elif event == cv2.EVENT_MOUSEMOVE:
            if drawing:
                cur_pos = (cx, cy)
                
        elif event == cv2.EVENT_LBUTTONUP:
            if not drawing: return
            drawing = False
            cur_pos = (cx, cy)
            
            x0, y0 = min(start_pos[0], cur_pos[0]), min(start_pos[1], cur_pos[1])
            x1, y1 = max(start_pos[0], cur_pos[0]), max(start_pos[1], cur_pos[1])
            px_w = x1 - x0
            px_h = y1 - y0

            # If it's just a click (or very tiny drag), treat as click-to-measure on object
            is_click = px_w < 5 and px_h < 5
            
            det_found = False
            det_label = ""
            if is_click:
                for det in detections:
                    dx0, dy0, dx1, dy1, label, conf, mask = det
                    if dx0 <= cx <= dx1 and dy0 <= cy <= dy1:
                        px_w = dx1 - dx0
                        px_h = dy1 - dy0
                        det_label = label
                        det_found = True
                        break
            
            # If we clicked but missed any detection, do nothing
            if is_click and not det_found:
                return

            def on_tof_received(dist):
                nonlocal calibrate_mode, last_dims
                if dist < 30 or dist > 2500:
                    last_dims = f"Error: Invalid ToF distance ({dist}mm)"
                    print(f"[ERR] {last_dims}")
                    return

                label_name = det_label if (is_click and det_found) else "Manual"

                if calibrate_mode:
                    if px_w > px_h:
                        real_w, real_h = A4_H_MM, A4_W_MM
                    else:
                        real_w, real_h = A4_W_MM, A4_H_MM
                        
                    focal_w = px_w * dist / real_w
                    focal_h = px_h * dist / real_h
                    avg_focal = (focal_w + focal_h) / 2.0
                    
                    save_calibration(avg_focal)
                    last_dims = f"CALIBRATED OK: Focal={avg_focal:.1f}"
                    send_display(real_w, real_h, dist, "Calib OK")
                    calibrate_mode = False
                    
                else:
                    if focal_px <= 0:
                        last_dims = "Error: Calibrate first!"
                        return
                        
                    ppm = focal_px / dist
                    real_w = px_w / ppm
                    real_h = px_h / ppm
                    
                    last_dims = f"{label_name}: {real_w:.1f} x {real_h:.1f} mm | Dist: {dist}mm"
                    print(f"[MEAS] {last_dims}")
                    send_display(real_w, real_h, dist, label_name)
                    send_buzz(2, 100)  # Beep on measurement

            # Fire async ToF fetch
            last_dims = "Measuring..."
            get_tof(on_tof_received)

    # Background YOLO Thread
    running = True
    def yolo_worker():
        nonlocal detections
        while running:
            if latest_frame is not None:
                # Process copy of latest frame
                frame_to_process = latest_frame.copy()
                results = model(frame_to_process, verbose=False, conf=0.35)
                new_dets = []
                if results and len(results) > 0:
                    result = results[0]
                    if result.boxes is not None:
                        for i, box in enumerate(result.boxes):
                            x0, y0, x1, y1 = box.xyxy[0].cpu().numpy().astype(int)
                            conf = float(box.conf[0])
                            label = model.names[int(box.cls[0])]
                            mask = result.masks[i].data[0].cpu().numpy() if (result.masks and i < len(result.masks)) else None
                            new_dets.append((x0, y0, x1, y1, label, conf, mask))
                with det_lock:
                    detections = new_dets
            time.sleep(0.05)  # Max ~20 FPS for YOLO to avoid CPU hogging

    yolo_thread = threading.Thread(target=yolo_worker, daemon=True)
    yolo_thread.start()

    # Start button polling thread
    btn_thread = threading.Thread(target=button_poller, daemon=True)
    btn_thread.start()

    cv2.namedWindow("ESP32 YOLO Measure", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("ESP32 YOLO Measure", 960, 720)
    cv2.setMouseCallback("ESP32 YOLO Measure", on_mouse, {"frame_shape": (720, 960, 3)})

    while True:
        ret, frame = cap.read()
        if not ret:
            if latest_frame is None:
                display = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(display, "Connecting to stream...", (50, 240), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 150, 255), 2)
            else:
                display = latest_frame.copy()
                cv2.putText(display, "STREAM RECONNECTING...", (20, 50), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        else:
            latest_frame = frame
            display = frame.copy()

        # Get latest detections safely
        with det_lock:
            current_dets = detections.copy()

        # ─── Auto-measure: track object persistence ───────
        now = time.time()
        current_labels = set()
        for det in current_dets:
            x0, y0, x1, y1, label, conf, mask = det
            bbox = (x0, y0, x1, y1)
            current_labels.add(label)

            if label in tracked_objects:
                # Check if same object (IoU > 0.3 = same position)
                if iou(tracked_objects[label]["bbox"], bbox) > 0.3:
                    tracked_objects[label]["bbox"] = bbox  # update current position
                    # Keep the BEST detection (highest confidence) for measurement
                    if conf > tracked_objects[label]["best_conf"]:
                        tracked_objects[label]["best_conf"] = conf
                        tracked_objects[label]["best_bbox"] = bbox
                else:
                    # Object moved significantly — reset everything
                    tracked_objects[label] = {
                        "first_seen": now, "bbox": bbox,
                        "best_bbox": bbox, "best_conf": conf
                    }
            else:
                tracked_objects[label] = {
                    "first_seen": now, "bbox": bbox,
                    "best_bbox": bbox, "best_conf": conf
                }

        # Remove objects no longer detected
        for label in list(tracked_objects.keys()):
            if label not in current_labels:
                del tracked_objects[label]

        # Check if any object has been held for HOLD_TIME
        if not calibrate_mode and not auto_measuring and focal_px > 0 and not measure_locked:
            for label, info in tracked_objects.items():
                hold_duration = now - info["first_seen"]
                # Check cooldown
                if label in auto_measured and (now - auto_measured[label]) < COOLDOWN_TIME:
                    continue
                if hold_duration >= HOLD_TIME:
                    # Use the BEST detection's bbox (highest confidence seen)
                    bx0, by0, bx1, by1 = info["best_bbox"]
                    a_px_w = bx1 - bx0
                    a_px_h = by1 - by0
                    a_label = label
                    a_conf = info["best_conf"]
                    auto_measuring = True

                    def on_auto_tof(dist, lbl=a_label, pw=a_px_w, ph=a_px_h, cf=a_conf):
                        nonlocal last_dims, auto_measuring
                        auto_measuring = False
                        if dist < 30 or dist > 2500:
                            last_dims = f"Auto: Invalid ToF ({dist}mm)"
                            return
                        ppm = focal_px / dist
                        rw = pw / ppm
                        rh = ph / ppm
                        last_dims = f"[AUTO] {lbl}: {rw:.1f} x {rh:.1f} mm | Dist: {dist}mm (conf:{cf:.0%})"
                        print(f"[AUTO-MEAS] {last_dims}")
                        send_display(rw, rh, dist, lbl)
                        send_buzz(2, 100)  # Beep on auto-measurement
                        auto_measured[lbl] = time.time()

                    last_dims = f"Auto-measuring {a_label} (best conf: {a_conf:.0%})..."
                    get_tof(on_auto_tof)
                    break  # Only auto-measure one object at a time

        # Draw detections
        for det in current_dets:
            x0, y0, x1, y1, label, conf, mask = det
            color = tuple(int(c) for c in colors[hash(label) % 100])

            if mask is not None:
                mask_res = cv2.resize(mask, (display.shape[1], display.shape[0]))
                c_mask = np.zeros_like(display)
                c_mask[:] = color
                m_bool = mask_res > 0.5
                display[m_bool] = cv2.addWeighted(display[m_bool], 0.6, c_mask[m_bool], 0.4, 0)
                
                m_uint8 = m_bool.astype(np.uint8) * 255
                contours, _ = cv2.findContours(m_uint8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours(display, contours, -1, color, 2)

            text = f"{label} {conf:.0%}"
            cv2.putText(display, text, (x0 + 2, y0 - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3)
            cv2.putText(display, text, (x0 + 2, y0 - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            # Draw hold progress indicator
            if label in tracked_objects and not calibrate_mode:
                hold_dur = now - tracked_objects[label]["first_seen"]
                in_cooldown = label in auto_measured and (now - auto_measured[label]) < COOLDOWN_TIME
                if hold_dur > 0.2 and hold_dur < HOLD_TIME and not in_cooldown:
                    # Progress arc: fills as hold approaches HOLD_TIME
                    progress = min(hold_dur / HOLD_TIME, 1.0)
                    h_disp, w_disp = display.shape[:2]
                    sx = w_disp / CAM_W
                    sy = h_disp / CAM_H
                    cx_arc = int((x0 + x1) / 2 * sx)
                    cy_arc = int((y0 + y1) / 2 * sy)
                    radius = 20
                    angle = int(progress * 360)
                    cv2.ellipse(display, (cx_arc, cy_arc), (radius, radius),
                                -90, 0, angle, (0, 255, 100), 3)
                    cv2.putText(display, f"{progress:.0%}", (cx_arc - 12, cy_arc + 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 100), 1)
                elif in_cooldown:
                    # Show checkmark for recently measured
                    h_disp, w_disp = display.shape[:2]
                    sx, sy = w_disp / CAM_W, h_disp / CAM_H
                    cx_m = int((x0 + x1) / 2 * sx)
                    cy_m = int((y0 + y1) / 2 * sy)
                    cv2.putText(display, "Measured", (cx_m - 30, cy_m + 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (100, 255, 150), 2)

        # Draw manual bounding box if currently dragging
        if drawing:
            h_disp, w_disp = display.shape[:2]
            scale_x, scale_y = w_disp / CAM_W, h_disp / CAM_H
            dx0 = int(start_pos[0] * scale_x)
            dy0 = int(start_pos[1] * scale_y)
            dx1 = int(cur_pos[0] * scale_x)
            dy1 = int(cur_pos[1] * scale_y)
            
            # Draw semi-transparent fill
            overlay = display.copy()
            cv2.rectangle(overlay, (dx0, dy0), (dx1, dy1), (200, 100, 255), -1)
            cv2.addWeighted(overlay, 0.3, display, 0.7, 0, display)
            
            # Draw dashed border
            cv2.rectangle(display, (dx0, dy0), (dx1, dy1), (255, 150, 255), 2)

        # Status bar
        h, w = display.shape[:2]
        cv2.rectangle(display, (0, h - 40), (w, h), (20, 20, 30), -1)

        if calibrate_mode:
            cv2.putText(display, "CALIBRATE: Click A4 sheet or drag box", (10, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)
        elif measure_locked:
            # Show lock indicator + frozen measurement
            cv2.rectangle(display, (0, h - 40), (130, h), (0, 0, 180), -1)
            cv2.putText(display, "LOCKED", (10, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            if last_dims:
                cv2.putText(display, last_dims, (140, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (100, 255, 150), 1)
        elif last_dims:
            cv2.putText(display, last_dims, (10, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (100, 255, 150), 1)
        else:
            cv2.putText(display, "Hold object 2s=auto | Click/drag=manual | 'c'=calib", (10, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 180), 1)

        cv2.setMouseCallback("ESP32 YOLO Measure", on_mouse, {"frame_shape": display.shape})
        cv2.imshow("ESP32 YOLO Measure", display)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'): 
            running = False
            break
        elif key == ord('c'):
            calibrate_mode = True
            last_dims = ""
            print("[MODE] Calibrate — click A4 sheet")
        elif key == ord('l'):
            measure_locked = not measure_locked
            print(f"[KEY] {'LOCKED' if measure_locked else 'UNLOCKED'}")
        elif key == ord('r'):
            print("[REFRESH] Reconnecting...")
            cap.release()
            time.sleep(0.5)  # Let old stream clean up
            latest_frame = None
            tracked_objects.clear()
            auto_measured.clear()
            cap = MJPEGStream(STREAM_URL)
            last_dims = "Stream reconnecting..."

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    if len(sys.argv) > 1:
        ESP32_IP = sys.argv[1]
        STREAM_URL = f"http://{ESP32_IP}:81"
        TOF_URL = f"http://{ESP32_IP}/tof"
        DISPLAY_URL = f"http://{ESP32_IP}/display"
        BUZZ_URL = f"http://{ESP32_IP}/buzz"
        BUTTON_URL = f"http://{ESP32_IP}/button"
        print(f"Using ESP32 IP: {ESP32_IP}")
    main()
