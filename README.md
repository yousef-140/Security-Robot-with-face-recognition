# 🤖 ESP32-CAM Security Robot

A WiFi-controlled mobile surveillance robot built using the ESP32-CAM module.  
The system integrates real-time video streaming, face detection, obstacle avoidance, and remote motor control into a single embedded platform.

---

## 🚀 Features

- 📡 WiFi-based control via browser
- 🎥 Live MJPEG video streaming
- 🧠 On-device Face Detection (no cloud required)
- 🔊 Buzzer alert when a face is detected
- 🕹️ Full motor control (Forward / Backward / Left / Right / Stop)
- 🚧 Obstacle avoidance using Ultrasonic sensor (HC-SR04)
- 🌐 Web-based control interface
- ⚙️ Fully embedded system (no external server)

---

## 🧠 System Architecture

The system is divided into modular components:

- Camera Module → Captures frames
- Web Server → Handles HTTP requests
- Motor Control → Controls movement
- Ultrasonic Sensor → Prevents collisions
- Face Detection Module → Detects faces
- Buzzer Module → Alerts on detection

---

## 🔌 Hardware Components

| Component | Description |
|----------|------------|
| ESP32-CAM | Main controller |
| L298N | Motor driver |
| DC Motors | Movement |
| HC-SR04 | Distance sensor |
| Buzzer | Alert |
| Power Supply | 5V |

---

## ⚙️ How It Works

1. ESP32 connects to WiFi  
2. Starts HTTP server  
3. User opens IP in browser  
4. Sends command  
5. Robot moves  
6. Sensor prevents collision  
7. Face detection runs (if enabled)  

---

## ⚠️ Limitations

- No face recognition  
- Limited processing power  
- Possible lag during detection  

---

## 🔮 Future Work

- Face recognition  
- Mobile app  
- Better sensors  
- Autonomous navigation  

---

## Matrix Team 🤖
