# SLAM-Robot-project

Autonomous mobile robot project implementing 2D LiDAR SLAM and sensor fusion (LiDAR + IMU).

## Overview

This repository hosts the software stack for an autonomous mobile robot platform:
- **Sensor Drivers & Demos**: Standalone acquisition, parsing, and visualization tools (LD06 LiDAR).
- **ROS 2 Integration**: Modular workspace for Odometry, Sensor Fusion (EKF), and SLAM (Cartographer / SLAM Toolbox).
- **Embedded Control**: Microcontroller interface for motor control and IMU reading.

---

## Hardware Architecture

- **LiDAR**: DFRobot / LDROBOT LD06 (230 400 bauds, 360° FOV, 12m range, 10Hz scan rate)
- **IMU**: 6-DOF / 9-DOF Inertial Measurement Unit
- **Controller**: Raspberry Pi / Single Board Computer running Linux + ROS 2
- **Chassis**: Differential drive robot base

---

## Project Structure

```text
SLAM-Robot-project/
├── .gitignore
├── README.md
├── lidar_visualizer.py      # Standalone real-time radar visualizer for LD06 LiDAR
└── ros2_ws/                 # ROS 2 workspace (under development)
    └── src/
```

---

## Quickstart: LD06 Real-Time Visualizer

The visualizer connects directly to the LD06 serial stream, validates incoming packets with CRC8, transforms polar coordinates to metric cartesian space, and renders a live radar display with Pygame.

### Prerequisites

Ensure Python 3 is installed with `pyserial` and `pygame`:

```bash
pip install pyserial pygame
```

Ensure your user has read/write permissions for the serial port:
```bash
sudo usermod -aG dialout $USER
# (Log out and log back in for changes to take effect, or use: sudo chmod 666 /dev/ttyUSB0)
```

### Usage

Run with default options (`/dev/ttyUSB0` at 230,400 baud):
```bash
python3 lidar_visualizer.py
```

Optional arguments:
```bash
python3 lidar_visualizer.py --port /dev/ttyUSB0 --baud 230400 --scale 100 --window-size 800
```

Controls:
- `ESC` or close window to exit cleanly.
