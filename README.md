# ROSBOT Project

This project involves building a ROS2-based robot using a Raspberry Pi 5, a camera, LiDAR, various sensors, and a motor driver. The setup utilizes Docker to manage the environment.

## Components

- Raspberry Pi 5 (8GB RAM)
- Raspberry Pi NoIR Camera V2
- 64GB SD card
- LDRobot Lidar LD19
- 4 x Traxxas Tires & Wheels
- BMP280 temperature, pressure, and altitude sensor
- 3 x 18650 batteries
- DC-DC voltage converter
- 4-Channel Encoder Motor Driver Module (STM32F030)
- DFRobot TCS3200 Color Sensor

## Prerequisites

- Docker installed on your system
- Docker Compose installed on your system
- Raspberry Pi set up with Raspbian OS
- Camera and other sensors connected to the Raspberry Pi

## Installation

### Step 1: Clone the Repository

Create a directory structure for the ROS2 workspace:

```bash
git clone https://github.com/Robocamp-ry/rosbot.git
cd ~/rosbot
```

### Step 2: Build the Docker Container

```bash
sudo docker-compose up --build
```

### Step 3: Access the Container

```bash
sudo docker exec -it rosbot bash
```

## Conclusion

Your ROSBOT should now be up and running with all components integrated. This setup provides a robust environment to develop and test your robot's capabilities using ROS2 and Docker.