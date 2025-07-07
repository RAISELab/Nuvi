# NUVI: Congestion-aware Autonomous Mobility for Airports

[![License: Apache 2.0](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)

---

## Overview

NUVI is an autonomous mobility system designed for airport environments, supporting congestion-aware path planning, ETA prediction, and intuitive visual feedback using projection UI.  
The system leverages real-time crowd density data to optimize navigation and enhance user experience for all passengers.

---

## Features

- Real-time path planning with congestion avoidance
- Dynamic ETA prediction based on crowd density
- Visual feedback with onboard projector
- Web UI for destination selection
- Modular software architecture (ROS2-based)

---

## Getting Started

### Prerequisites

- Ubuntu 22.04 or later
- ROS2 Humble
- [Intel RealSense SDK](https://github.com/IntelRealSense/librealsense)
- Python 3.8+
- (Optional) MQTT broker

### Installation

```bash
# Clone repository
git clone https://github.com/RAISELab/Nuvi.git
cd Nuvi

# Install dependencies (example for ROS2)
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build
source install/setup.bash
