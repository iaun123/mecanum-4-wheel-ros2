# 🤖 Real Robot Deployment (Jetson Nano)

This directory contains the production ROS 2 packages and drivers for running the physical 4-wheel Mecanum robot on an **NVIDIA Jetson Nano**.

---

## 🔌 Hardware Specifications

- **Chassis:** 4-wheel independent Mecanum drive system (Omnidirectional)
- **Main Processor:** NVIDIA Jetson Nano Mini PC
- **Sensors:**
  - **Wheel Encoders:** High-resolution wheel odometry and velocity feedback
  - **IMU:** 9-DOF Inertial Measurement Unit for orientation tracking and EKF fusion
  - **2D LiDAR:** Laser scan for 2D mapping and obstacle avoidance
  - **3D RGB-D Camera:** Depth perception for 3D point cloud mapping and RTAB-Map

---

## 📦 Launch Scripts Reference

| Launch Script | Description |
| :--- | :--- |
| `2d_map.launch.py` | 2D SLAM mapping using SLAM Toolbox on the physical robot. |
| `2d_nav.launch.py` | 2D Nav2 autonomous navigation and obstacle avoidance. |
| `2d_waypoint.launch.py` | Automated multi-waypoint patrol sequences in 2D. |
| `3d_nav.launch.py` | 3D Visual SLAM and Nav2 navigation with RGB-D camera. |
| `3d_waypoint.launch.py` | 3D Waypoint navigation using RTAB-Map. |
| `rtab_map.launch.py` | RTAB-Map 3D visual graph SLAM and point cloud mapping. |
| `occupancy_grid.launch.py` | 2D occupancy grid generation from 3D point cloud. |
