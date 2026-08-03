# Vehicle-to-Vehicle (V2V) Radar Application

[![Flutter](https://img.shields.io/badge/Flutter-02569B?style=flat-square&logo=flutter&logoColor=white)](https://flutter.dev)
[![Dart](https://img.shields.io/badge/Dart-0175C2?style=flat-square&logo=dart&logoColor=white)](https://dart.dev)
[![Platform](https://img.shields.io/badge/Platform-Android%20%7C%20iOS-lightgrey?style=flat-square)]()
[![License](https://img.shields.io/badge/License-MIT-blue?style=flat-square)](LICENSE)

## Overview

The **Vehicle-to-Vehicle (V2V) Radar Application** is a mobile platform developed with Flutter designed for real-time spatial visual tracking and proximity alerts between nearby vehicles. The system captures spatial positioning, telemetry, and relative distances, presenting them on an integrated radar dashboard to assist in situational awareness.

---

## System Features

* **Real-Time Spatial Detection:** Continuously processes location vectors to locate nearby participating vehicles relative to the user's position.
* **Radar Interface:** Displays proximity vectors, target headings, and relative speed indications on a custom radar UI optimized for landscape display.
* **Proximity & Collision Alerting:** Triggers immediate feedback and continuous haptic alerts upon detecting critical distance thresholds or emergency hazard signals.
* **Peer-to-Peer Communication:** Utilizes low-latency hardware mesh protocols for localized data transmission without reliance on cellular networks.

---

## Technical Architecture

* **UI Framework:** Flutter / Dart (Landscape Dashboard Interface)
* **State Management:** Reactive State Management (optimized for high-frequency telemetry)
* **Hardware & Sensor Processing:** 
  * Integrated Device GPS, Compass, and Gyroscope
  * Kalman Filtering algorithm for telemetry data smoothing and noise reduction
* **V2V Communication & Mesh Networking:**
  * **Hardware Node:** ESP32 Microcontrollers
  * **P2P Protocol:** ESP-NOW protocol for low-latency vehicle-to-vehicle mesh networking
  * **Mobile Interface:** Bluetooth Serial / BLE for ESP32-to-Flutter telemetry streaming
* **Safety & Hazard System:**
  * Collision detection algorithm with real-time payload parsing
  * Priority SOS emergency hazard broadcasting with haptic feedback

---

## Getting Started

### Prerequisites

* Flutter SDK (`version 3.0.0` or higher)
* Dart SDK
* Android Studio / Xcode configured with emulator or physical test devices
* ESP32 Development Board (flashed with ESP-NOW V2V firmware)

### Installation and Execution

1. **Clone the Repository:**
   ```bash
   git clone [https://github.com/Saurav-Kiron/v2v_radar_app.git](https://github.com/Saurav-Kiron/v2v_radar_app.git)
   cd v2v_radar_app
