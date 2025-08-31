# Omni-Directional Robot Control System

## Overview
This project implements a control system for an omnidirectional (holonomic) robot platform based on the ESP32 microcontroller. The system provides precise control of a three-wheeled omnidirectional robot with integrated orientation sensing, motor control, and wireless communication capabilities.

## Features
- **Omnidirectional Movement**: Precise control in any direction (X, Y, and rotational)
- **BNO055 IMU Integration**: Orientation sensing and calibration
- **PID Control**: Closed-loop velocity control for each motor
- **WiFi Connectivity**: Remote control and telemetry via WiFi
- **OTA Updates**: Over-The-Air firmware updates
- **Real-time Feedback**: Speed and orientation feedback
- **Configurable Parameters**: Easily adjustable system parameters

## Hardware Requirements
- ESP32 Development Board
- BNO055 IMU Sensor
- 3 DC Motors with Encoders (1980 pulses per revolution)
- Motor Drivers
- Power Supply
- Omnidirectional Wheel Assembly

## Software Requirements
- ESP-IDF (ESP32 Development Framework)
- Python 3

## Setup Instructions
1. **Clone the Repository**
   ```
   git clone [repository-url]
   cd Omni
   ```

2. **Configure WiFi Settings**
   Edit `main/inc/sys_config.h` with your WiFi credentials and server IP:
   ```c
   #define WIFI_SSID "your_ssid"
   #define WIFI_PASS "your_password"
   #define SERVER_IP "your_server_ip"
   ```

## Usage
1. **Basic Control**
   - Send movement commands via WiFi socket connection
   - Format: JSON commands for velocity and direction

2. **Calibration**
   - The BNO055 IMU requires calibration before accurate readings
   - System reports calibration status via socket connection

3. **OTA Updates**
   - Send "Upgrade" command to the device to enter OTA mode

4. **PID Tuning**
   - Send "Set PID" command followed by parameter values

## System Architecture
- **Motor Control**: Direct control of 3 motors with encoder feedback
- **Sensor Integration**: BNO055 IMU for orientation
- **Communication**: WiFi socket-based interface with server
- **Motion Control**: Velocity and direction calculations for omnidirectional movement

## Configuration
Key parameters can be adjusted in `sys_config.h` and other header files:
- Robot dimensions in `omni_control.h`
- PID parameters
- Communication settings
- IMU configuration

## Troubleshooting
- **Connection Issues**: Verify WiFi credentials and server IP
- **Movement Problems**: Check encoder connections and motor wiring
- **Orientation Drift**: Ensure proper IMU calibration
