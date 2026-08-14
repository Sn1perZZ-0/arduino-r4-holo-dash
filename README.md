# Arduino Uno R4 WiFi MPU6050 3D HUD Dashboard

A real-time 3D orientation tracking system powered by an **Arduino Uno R4 WiFi** and an **MPU6050 IMU**, featuring a cyberpunk-inspired **Three.js WebGL HUD** and an onboard **OLED local display**.

---

## ✨ Features

* **Live WebGL 3D Visualization:** A responsive browser-based dashboard built with Three.js that renders a holographic 3D representation of your sensor setup, reacting instantly to physical tilt and rotation.
* **Built-in HTTP Telemetry Server:** The Arduino acts as a local web server, serving live roll and pitch telemetry as JSON data to any connected browser on your local network.
* **Onboard OLED Feedback:** Features a 0.96" SSD1306 I2C OLED display showing real-time numerical orientation values alongside a custom real-time 3D wireframe cuboid.
* **Accurate Sensor Fusion:** Implements a complementary filter combining accelerometer and gyroscope inputs for smooth, drift-free angle calculations.
* **LED Matrix Status Animation:** Uses the Arduino Uno R4's built-in LED matrix to display custom animations during WiFi connection sequences.

---

## 🛠️ Hardware Requirements

* **Microcontroller:** Arduino Uno R4 WiFi
* **Sensor:** MPU6050 6-Axis Accelerometer/Gyroscope
* **Display:** 0.96" I2C SSD1306 OLED Display (128x64)
* Breadboard and jumper wires

---

## 🔌 Wiring Guide

| Component | Pin | Arduino Uno R4 WiFi Pin |
| --- | --- | --- |
| **MPU6050** | VCC | 5V |
|  | GND | GND |
|  | SDA | SDA (A4) |
|  | SCL | SCL (A5) |
| **OLED Display** | VCC | 3.3V / 5V |
|  | GND | GND |
|  | SDA | SDA (A4) |
|  | SCL | SCL (A5) |

---

## 🚀 Quick Start

1. **Clone or Download:** Download this repository or clone it using Git.
2. **Dependencies:** Install the following libraries via the Arduino IDE Library Manager:
* `Adafruit GFX Library`
* `Adafruit SSD1306`
* `Arduino_LED_Matrix`
* `WiFiS3`


3. **Configuration:** Open the sketch (`.ino`) and update your local Wi-Fi credentials:
```cpp
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

```


4. **Upload:** Flash the code to your Arduino Uno R4 WiFi.
5. **Monitor:** Open the Arduino Serial Monitor (115200 baud) to view the assigned local IP address.
6. **View Dashboard:** Open any browser on a device connected to the same network, navigate to `http://<YOUR_ARDUINO_IP>`, and tilt your sensor to watch the live 3D HUD react!

---

## 📜 License

Distributed under the **MIT License**. See `LICENSE` for more information.
