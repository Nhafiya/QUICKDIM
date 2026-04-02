 QUICKDIM – Smart Dimension Measuring Spectacle


Overview
QUICKDIM is a smart wearable system that measures real-world object dimensions using computer vision and a Time-of-Flight sensor.



Components Used
- ESP32-CAM (OV3660)
- VL53L0X ToF Sensor
- OLED Display (SSD1306)
- Li-Po Battery



 Working Principle
1. Camera captures image  
2. YOLO detects object → gives pixel size  
3. ToF sensor measures distance  
4. Real-world size is calculated  
5. Output is displayed on OLED  



 Mathematical Model
Real Size = (Pixel Size × Distance) / Focal Length  



  System Architecture
- ESP32 → image capture + sensor data  
- PC → YOLO processing + dimension calculation  
- OLED → displays result  



  Features
- Real-time measurement  
- Hands-free operation  
- Portable and lightweight  



  Limitations
- Requires calibration  
- Affected by lighting conditions  
- Errors for tilted objects  



  Technologies Used
- Python (YOLO, OpenCV)
- Arduino (ESP32)
