# 🪖 Smart Helmet with Sensor-Based Ignition Control and Accident Detection System

This repository contains the **Smart Helmet with Sensor-Based Ignition Control and Accident Detection System**, an embedded systems project designed to improve two-wheeler rider safety by actively preventing unsafe riding conditions and enabling automatic emergency alerts.

The project integrates **ESP32 microcontrollers, sensors, and wireless communication** to enforce helmet usage, detect alcohol consumption, and respond to accidents in real time.

---

## 🚦 Problem Statement

Two-wheeler accidents frequently occur due to:
- Riding without a helmet
- Driving under the influence of alcohol
- Delay in medical assistance after accidents

Conventional helmets provide only passive protection.  
This project proposes a **Smart Helmet System** that actively **prevents**, **detects**, and **responds** to unsafe riding conditions.

---

## ✨ Key Features

- ✅ Helmet wearing detection using **Limit Switch**
- 🍺 Alcohol detection using **MQ-3 Sensor**
- 🔐 Vehicle ignition control using **Relay Module**
- 🚑 Accident detection using **MPU6050 (Accelerometer + Gyroscope)**
- 📍 Real-time location tracking using **GPS Neo-6M**
- 📩 Automatic emergency SMS alerts using **GSM SIM900A**
- 📟 System status display on **16×2 I2C LCD**
- 📡 Wireless communication between helmet and bike using **ESP-NOW**
- 🔋 Rechargeable battery-powered system

---

## 🧠 System Architecture

The system is divided into **two independent units**:

### 🔹 Helmet Unit (Transmitter)
- ESP32 microcontroller
- MQ-3 alcohol sensor
- Limit switch for helmet detection
- 18650 Li-ion battery with TP4056 charging module
- Sends helmet status and alcohol data wirelessly to bike unit

### 🔹 Bike Unit (Receiver)
- ESP32 microcontroller
- MPU6050 for accident detection
- GPS Neo-6M for location tracking
- GSM SIM900A for emergency alerts
- Relay module for ignition control
- 7805 voltage regulator
- 16×2 LCD for system feedback

---

## ⚙️ Working Principle

1. Rider wears the helmet → limit switch is pressed  
2. MQ-3 sensor checks alcohol concentration  
3. Helmet ESP32 sends data to bike ESP32 using ESP-NOW  
4. If safety conditions are satisfied → ignition relay is enabled  
5. MPU6050 continuously monitors motion and orientation  
6. On accident detection:
   - GPS fetches real-time coordinates  
   - GSM sends emergency SMS with Google Maps location link  

---

## 📩 Emergency Alert Format
Accident Alert!
Location: http://maps.google.com/maps?q=
<latitude>,<longitude>

---

## 🛠️ Hardware Components Used

- ESP32 Development Boards (Helmet & Bike units)
- MQ-3 Alcohol Sensor
- Limit Switch
- MPU6050 Accelerometer & Gyroscope
- GPS Neo-6M Module
- GSM SIM900A Module
- Relay Module
- 16×2 LCD with I2C backpack
- 7805 Voltage Regulator
- 12V Power Supply (Bike Unit)
- 18650 Li-ion Battery
- TP4056 Charging Module
- Connecting wires & PCB

---

## 💻 Software & Tools

- Arduino IDE
- ESP32 Board Manager
- ESP-NOW Protocol
- TinyGPS++ Library
- MPU6050 Library
- LiquidCrystal_I2C Library
- HardwareSerial Library

---

## 📂 Repository Structure
Smart-Helmet-Ignition-Accident-Detection/
├── README.md
│
├── code/
│   ├── helmet_unit/
│   │   ├── helmet_unit.ino
│   │   └── README.md
│   │
│   └── bike_unit/
│       ├── bike_unit.ino
│       └── README.md
│
├── circuit_diagrams/
│   ├── helmet_circuit.png
│   └── bike_circuit.png
│
├── images/
│   ├── helmet_prototype.jpg
│   ├── bike_unit.jpg
│   └── working_demo.jpg
│
└── report/
    └── Smart_Helmet_Project_Report.pdf

## 🚧 Challenges & Limitations

- False positives due to high sensitivity of MPU6050
- GPS signal delay in indoor or dense urban environments
- GSM network dependency
- Limited battery backup during continuous operation

---

## 🔮 Future Enhancements

- Mobile application integration
- Cloud-based emergency alert system
- Machine learning-based accident classification
- Voice alerts inside helmet
- Camera-based accident recording

---

⭐ *If you find this project useful, please consider starring the repository.*


