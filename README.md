# Tele-operated Robot with SLAM Mapping

This project is part of the **Engineering Principles and Practices (CG2111A)** course. It involves building a remote-controlled robot capable of mapping its environment using SLAM (Simultaneous Localization and Mapping). The robot uses Raspberry Pi and Arduino microcontrollers, with several sensors for navigation and mapping.

## Project Overview

The robot navigates through an area, detects its surroundings using sensors, and creates a real-time map using SLAM. Remote control is achieved securely via TLS, and the system runs on Linux to manage control and data communication.

### Key Features:
- **SLAM for Mapping**
- **Color Sensor Detection**
- **Motorized Movement**
- **Secure Remote Control via TLS**
- **Linux-Based Environment (Raspberry Pi OS)**

## Hardware Components
- **Microcontrollers:**
  - Raspberry Pi
  - Arduino
- **Sensors:**
  - SLAM (e.g., LIDAR or Camera)
  - Color Sensor
  - Motors for movement

## Software Stack
- **Operating System:** Linux (Raspberry Pi OS)
- **Secure Communication:** TLS encryption for remote control
- **Unix Commands for Control**

## Setup and Installation

### Prerequisites
- Raspberry Pi with Linux OS
- Arduino with necessary libraries
- TLS for secure communication

## Layout
- Alex/ : Arduino Mega sketch and robot code
- pi/ : Raspberry Pi host program
  - pi/src/ : C++ sources
  - pi/include/ : C/C++ headers

## Build the Pi host program
From the repository root:

```bash
make
```

This builds the `Alex-pi` binary in the repo root.

## Run the Pi host program
```bash
./Alex-pi
```

## Arduino sketch
Open the sketch in `Alex/` using Arduino IDE or `arduino-cli`.
The board target in `Alex/sketch.yaml` is `arduino:avr:mega`.
