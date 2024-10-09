# Remote-Controlled Robot with SLAM Mapping

This project is part of the **Engineering Principles and Practices (CG2111A)** course. It involves building a remote-controlled robot capable of mapping its environment using SLAM (Simultaneous Localization and Mapping). The robot uses Raspberry Pi and Arduino microcontrollers, with several sensors for navigation and mapping.

## Project Overview

The robot navigates through an area, detects its surroundings using sensors, and creates a real-time map using SLAM. Remote control is achieved securely via TLS, and the system runs on Linux with Robot OS (ROS) managing the control and data communication.

### Key Features:
- **SLAM for Mapping**
- **Color Sensor Detection**
- **Motorized Movement**
- **Secure Remote Control via TLS**
- **Linux-Based Environment (Raspberry Pi OS)**
- **Robot OS (ROS) Framework**

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
- **Robot OS (ROS)**
- **Secure Communication:** TLS encryption for remote control
- **Unix Commands for Control**

## Setup and Installation

### Prerequisites
- Raspberry Pi with Linux OS
- Arduino with necessary libraries
- ROS installed on the Raspberry Pi
- TLS for secure communication

### Installation Steps:
1. Clone the repository:
   ```bash
   git clone <repository-url>
   cd <repository-directory>
