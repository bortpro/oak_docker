# ROS2 x Docker x RPi5 x Luxonis OAK Camera Setup

This project provides a Dockerized ROS2 Humble environment specifically configured for running Luxonis OAK-D-S2 camera applications on Raspberry Pi 5.

## Features

- **ROS2 Humble**: Latest stable ROS2 distribution
- **DepthAI Integration**: Pre-configured with DepthAI 2.30.0.0 for Luxonis OAK camera support
- **Computer Vision Ready**: Includes OpenCV, cv-bridge, and vision_msgs packages
- **RPi5 Optimized**: Built for Raspberry Pi 5 hardware requirements
- **Docker Isolation**: Consistent environment across development and deployment

## Prerequisites

- Raspberry Pi 5
- Docker installed on RPi5
- Luxonis OAK-D-S2 camera connected via USB

## What's Included

ROS2 Humble base with development tools
- DepthAI 2.30.0.0 Python bindings
- OpenCV and ROS2 computer vision packages
- Pre-configured workspace structure at `/ros2_ws

## Notes

- The workspace follows standard ROS2 layout (build/, install/, src/)
- Build occurs inside the container after mounting your source code
