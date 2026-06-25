# ros2_lidar_slam_bot

Autonomous ROS2-based mobile robot with LiDAR SLAM, sensor fusion, and Nav2 navigation.

## Overview

This project implements a fully autonomous indoor mobile robot built on ROS2.  
The system integrates LiDAR-based SLAM, wheel odometry, IMU sensor fusion, and autonomous navigation using the Nav2 stack.

The goal is to develop a modular robotics platform capable of:
- Real-time mapping (SLAM)
- State estimation via sensor fusion (EKF)
- Autonomous navigation and obstacle avoidance
- Scalable ROS2-based architecture

## System Capabilities

- 2D LiDAR SLAM (occupancy grid mapping)
- Wheel odometry + IMU integration
- Sensor fusion using Extended Kalman Filter (EKF)
- Autonomous path planning and navigation (Nav2)
- Modular ROS2 node architecture
- Real-time visualization in RViz2

## Hardware (Planned / In Progress)

- Differential drive mobile base
- 2D LiDAR sensor
- Wheel encoders
- IMU
- ESP32-based low-level motor controller
- Optional: front-facing camera for visual experiments

## Software Stack

- Ubuntu 24.04 LTS
- ROS2 (Jazzy)
- C++ / Python
- SLAM Toolbox
- robot_localization (EKF)
- Nav2
- OpenCV (for vision experiments)
- PlatformIO (firmware layer)


