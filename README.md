Remote-Controlled Robot with SLAM Mapping

This project is part of the Engineering Principles and Practices (CG2111A) course and involves building a non-trivial robot capable of autonomously navigating and mapping its environment using SLAM (Simultaneous Localization and Mapping). The robot is controlled remotely and uses various sensors to traverse and generate a map of the area covered.

Project Overview

The robot is built using a combination of microcontroller boards and sensors that allow it to detect its surroundings, navigate through obstacles, and create a map. The core components of this project include the Raspberry Pi and Arduino boards, which handle the control logic and sensor integration.

Features:
SLAM (Simultaneous Localization and Mapping): Enables the robot to map its surroundings as it navigates.
Color Sensor Integration: Detects environmental features such as color for further analysis.
Motor Control: Ensures precise movement and direction control during navigation.
Remote Control Capability: The robot can be remotely controlled through a secure connection.
Robot OS (ROS) Integration: Provides the framework for robot communication and data handling.
TLS for Secure Communication: Ensures encrypted data transmission between the robot and the remote controller.
Linux-based OS: The robot operates on Linux, providing a robust and flexible environment for programming and control.
Hardware Components

Microcontroller Boards:
Raspberry Pi
Arduino
Sensors and Peripherals:
SLAM (LIDAR or Camera)
Color Sensor
Motors for movement
Software Stack

Operating System: Linux (Raspberry Pi OS)
Robot OS (ROS): Used for robot communication, sensor data handling, and control.
Secure Communication: Implemented using TLS to ensure a secure connection for remote control.
Unix Commands: Various Unix-based commands are used for system control and task automation.
Installation and Setup

Prerequisites:
A working Raspberry Pi running a Linux-based OS.
Arduino IDE and the necessary libraries for motor and sensor control.
ROS (Robot Operating System) installed on the Raspberry Pi.
TLS libraries for securing communication.
