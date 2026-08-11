# MiniM5Robo

MiniM5Robo is a PlatformIO-based firmware project for an ESP32-S3 powered robot built around an M5Stack CoreS3 device. It integrates wireless communication, micro-ROS topics, sensor input, motor control, and OTA updates.

## Overview

This project is designed to run on an M5Stack CoreS3 and communicate with a ROS 2 environment over Wi-Fi using micro-ROS. It supports:

- Wi-Fi and OTA update support
- micro-ROS publishers and subscribers
- IMU, LiDAR, camera, and light sensor input
- Dynamixel motor control
- SD card based configuration loading

## Hardware and Software

### Hardware

- M5Stack CoreS3
- ESP32-S3 based controller
- Built-in display and buttons
- LiDAR, camera, IMU, and environmental sensors
- Dynamixel motors

### Software

- PlatformIO
- Arduino framework
- ESP32 / ESP-IDF compatible environment
- micro-ROS
- ROS 2 topics such as `/cmd_vel`, `/odom`, `/scan`, `/imu`, and `/camera/compressed_image`

## Project Structure

- [src/main.cpp](src/main.cpp) - main firmware entry point and task orchestration
- [include/](include/) - hardware drivers, robot control logic, and configuration headers
- [lib/](lib/) - additional libraries
- [platformio.ini](platformio.ini) - PlatformIO build and board configuration

## Configuration

Before building, configure your Wi-Fi and micro-ROS agent settings.

### Wi-Fi settings

Update the Wi-Fi credentials in [include/config/wifi_info.hpp](include/config/wifi_info.hpp).

### SD card configuration

The firmware can read network settings from a SD card configuration file at `/config.csv`.

## Build and Upload

From the project root, run:

```bash
platformio run
platformio run --target upload
```

You can also monitor serial output with:

```bash
platformio device monitor
```

## micro-ROS Setup

This project supports micro-ROS over Wi-Fi. The build configuration in [platformio.ini](platformio.ini) selects the ROS 2 distribution and transport.

### ROS 2 distribution options

Use Humble:

```ini
board_microros_distro = humble
board_microros_transport = wifi
```

Use Jazzy:

```ini
board_microros_distro = jazzy
board_microros_transport = wifi
```

## Host Setup

### Local host setup

Reference: https://github.com/micro-ROS/micro_ros_platformio

```bash
source /opt/ros/$ROS_DISTRO/setup.bash

mkdir microros_ws
cd microros_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

rosdep install --from-paths src --ignore-src -y
colcon build

source install/local_setup.bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
```

### Run the micro-ROS agent

```bash
source ~/microros_ws/install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v6
```

### Docker-based agent

```bash
docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:$ROS_DISTRO udp4 --port 8888 -v6
```

## OTA Update

The firmware supports over-the-air updates. Make sure the device is connected to Wi-Fi and that the OTA service is enabled.

## Notes

- Some settings such as agent IP, port, and Wi-Fi credentials may need to be adjusted for your local network.
- If the board does not connect as expected, verify the network configuration and ROS 2 agent availability.
