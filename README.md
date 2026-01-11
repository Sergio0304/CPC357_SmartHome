# 🏠 Smart Home IoT System  
**Smart Door Security System & AI Window Automation**

---

## 📌 Project Overview

This project implements a **Smart Home IoT System** consisting of two integrated modules:

1. **Smart Door Security System** – provides secure access control, intrusion detection, and smart lighting.
2. **AI Window System** – provides intelligent window control using **Edge AI voice recognition**, environmental sensing, and automation logic.

Both modules communicate through **MQTT** and can be monitored and controlled using a **Node-RED dashboard**, making the system suitable for **smart home**, **smart building**, and **smart city** applications.

---

## 🧩 System Modules

### 🔐 Smart Door Module
- NFC-based door access control
- Motion-triggered alarm system
- Smart lighting when occupants enter
- Remote locking and unlocking via MQTT

### 🪟 AI Window Module
- Voice-controlled window using Edge Impulse (offline)
- Automatic window control based on rain and temperature
- Manual button override
- MQTT-based monitoring and control

# 🔐 Smart Door Security System

## 🎯 Features

### 🔑 Access Control
- PN532 NFC authentication (SPI mode)
- Authorized UID whitelist
- NFC-based lock / unlock toggle
- Remote lock / unlock via MQTT

### 🚨 Security Alarm
- PIR motion detection
- Alarm triggers when door is **locked & armed**
- Red LED and buzzer alert
- Non-blocking beep-beep alarm pattern
- Automatic alarm timeout

### 💡 Smart Lighting
- Motion-triggered white LED when door is unlocked
- Automatic light timeout
- Light disabled when door is locked

---

## 🔧 Smart Door Hardware
- Arduino **UNO R4 WiFi**
- PN532 NFC module
- PIR motion sensor
- Continuous rotation servo motor
- Buzzer
- Red LED (alarm)
- White LED (smart light)

---

## 📡 Smart Door MQTT Topics

| Topic | Description |
|------|------------|
| `smartdoor/state` | Door state (`locked`, `unlocked`, `locking`) |
| `smartdoor/event` | NFC, alarm, and remote events |
| `smartdoor/alarm` | Alarm state (`armed`, `disarmed`, `alarm`) |
| `smartdoor/light` | Smart light state (`on`, `off`) |
| `smartdoor/cmd` | Commands (`lock`, `unlock`, `open`) |

---

# 🪟 AI Window System (Edge AI)

## 🎯 Features

### 🎙️ Voice Control (Edge AI)
- Offline keyword spotting using **Edge Impulse**
- Supported commands:
  - `open window`
  - `close window`
- Runs entirely on-device (no cloud inference)

### 🌧️ Environmental Automation
- Rain sensor → automatic window close
- DHT22 temperature sensor → auto-open when temperature ≥ 40°C
- Voice commands ignored while raining (safety lockout)

### 🖲️ Manual Control
- Physical push button to toggle window
- MQTT remote control support

### ⚙️ Servo Control
- Continuous servo motor
- Non-blocking open/close logic
- Cooldown and hysteresis to prevent repeated triggers

---

## 🔧 AI Window Hardware
- ESP32-S3 (Cytron Maker Feather AIoT S3)
- INMP441 I2S microphone
- Continuous rotation servo motor
- Rain sensor (digital output)
- DHT22 temperature sensor
- Push button

---

## 🧠 AI Window Intelligence Logic

### Priority Order
1. Rain safety (highest priority)
2. Temperature automation
3. Voice command
4. Manual button override

### Voice Detection Logic
- Confidence-based decision (open vs close)
- Hysteresis thresholds:
  - Trigger ON ≥ 0.70
  - Re-arm ≤ 0.50
- Cooldown and hold-off timers

---

## 📡 AI Window MQTT Topics

| Topic | Description |
|------|------------|
| `aiwindow/state` | Window state and sensor data |
| `aiwindow/event` | Actions and automation events |
| `aiwindow/cmd` | Remote commands |

---

## 📦 Dependencies & Requirements

### Hardware
- Arduino UNO R4 WiFi
- ESP32-S3
- PN532 NFC module
- INMP441 microphone
- PIR motion sensor
- Rain sensor
- DHT22 temperature sensor
- Continuous rotation servo motors
- LEDs, buzzer, resistors

### Software
- Arduino IDE v2.x
- Edge Impulse Studio (for model training)

### Arduino Libraries
- Adafruit PN532
- PubSubClient
- WiFiS3
- Servo
- SPI
- Edge Impulse Arduino SDK

---

## 🔐 Configuration (GitHub-Safe)

Create a local file `secrets.h` (DO NOT COMMIT):

```cpp
#define WIFI_SSID   "YOUR_WIFI_NAME"
#define WIFI_PASS   "YOUR_WIFI_PASSWORD"
#define MQTT_SERVER "YOUR_MQTT_BROKER_IP"
#define MQTT_PORT   1883



