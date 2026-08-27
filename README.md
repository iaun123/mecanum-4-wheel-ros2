<div align="center">

# 🤖 Autonomous 4-Wheel Mecanum Mobile Robot (ROS 2)

[![ROS 2](https://img.shields.io/badge/ROS_2-Foxy_%2F_Humble_%2F_Jazzy-22314E?style=for-the-badge&logo=ros&logoColor=white)](https://docs.ros.org/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-20.04_%2F_22.04_%2F_24.04-E95420?style=for-the-badge&logo=ubuntu&logoColor=white)](https://ubuntu.com/)
[![Nav2](https://img.shields.io/badge/Navigation-Nav2_Stack-00599C?style=for-the-badge&logo=navigation&logoColor=white)](https://navigation.ros.org/)
[![Gazebo](https://img.shields.io/badge/Simulation-Gazebo_Sim-FF6F00?style=for-the-badge&logo=gazebo&logoColor=white)](https://gazebosim.org/)
[![RTAB-Map](https://img.shields.io/badge/SLAM-RTAB--Map_%26_SLAM_Toolbox-3776AB?style=for-the-badge&logo=c%2B%2B&logoColor=white)](http://introlab.github.io/rtabmap/)

<p align="center">
  <b>An end-to-end Autonomous Mobile Robot (AMR) platform featuring 4-wheel omnidirectional Mecanum kinematics, 2D/3D visual SLAM, and Nav2 waypoint navigation across simulation and real hardware (NVIDIA Jetson Nano).</b><br>
  <i>(Mechatronics Engineering Technology - MtET Thesis Project)</i>
</p>

</div>

---

## 📌 Overview

This repository provides a full-stack robotics project developing an omnidirectional **Autonomous Mobile Robot (AMR)**. By leveraging independent 4-wheel **Mecanum drive kinematics**, the robot achieves 3-DOF planar mobility (omnidirectional translation and rotation) suitable for tight industrial corridors, indoor transport, and autonomous patrolling.

The project maintains **1:1 digital-twin parity** between:
1. **Simulation (`/simulation`):** High-fidelity physics modeling in **Gazebo**, custom residential/office environments, URDF/Xacro descriptions, and virtual sensor plugins.
2. **Real Hardware (`/real`):** Embedded deployment on an **NVIDIA Jetson Nano**, sensor fusion with 2D LiDAR, 3D Depth Camera, IMU, and motor encoders.

---

## ✨ Key Features

- 🔄 **Omnidirectional Kinematics:** Custom kinematics solver for 4-wheel independent Mecanum drive (forward/backward, sideways strafe, diagonal, in-place rotation).
- 🗺️ **2D & 3D Multi-Modal SLAM:**
  - **2D Mapping:** SLAM Toolbox & Cartographer for rapid laser-based 2D occupancy grid generation.
  - **3D Visual SLAM:** RTAB-Map utilizing RGB-D depth camera and point cloud processing (`pcl_ros`).
- 🧭 **Autonomous Waypoint Navigation (Nav2):** Integrated with ROS 2 Navigation Stack for global planning, local collision avoidance (DWB / TEB local planner), and autonomous multi-waypoint patrol sequences.
- 🎮 **Dual Operation Modes:** Seamless switching between autonomous mission execution and manual joystick teleoperation (PS4 controller).
- 🏗️ **Modular ROS 2 Architecture:** Clear separation of robot description, perception, navigation, hardware interface, and teleop packages.

---

## 🏗️ System Architecture

```mermaid
graph TD
    subgraph Sensing_Layer ["🔌 Sensing & Hardware Layer"]
        LIDAR["2D LiDAR (LaserScan)"]
        DEPTH_CAM["3D RGB-D Camera (PointCloud2 / Image)"]
        IMU["IMU (Orientation & Accel)"]
        ENC["Wheel Encoders"]
        JOY["PS4 Joy Controller"]
    end

    subgraph Localization_Fusion ["🧠 State Estimation & SLAM"]
        EKF["robot_localization (EKF Node)"]
        SLAM_2D["SLAM Toolbox (2D Occupancy Grid)"]
        RTAB["RTAB-Map (3D Point Cloud & Graph SLAM)"]
        IMU --> EKF
        ENC --> EKF
        LIDAR --> SLAM_2D
        DEPTH_CAM --> RTAB
        EKF --> SLAM_2D
    end

    subgraph Navigation_Planning ["🧭 Nav2 Autonomy Stack"]
        NAV2_BT["Nav2 Behavior Tree Navigator"]
        GLOBAL_PLAN["Global Costmap & Planner"]
        LOCAL_PLAN["Local Costmap & Controller"]
        WAYPOINTS["Waypoint Follower (2D / 3D)"]

        SLAM_2D --> GLOBAL_PLAN
        RTAB --> GLOBAL_PLAN
        GLOBAL_PLAN --> LOCAL_PLAN
        WAYPOINTS --> NAV2_BT
        NAV2_BT --> LOCAL_PLAN
    end

    subgraph Motion_Actuation ["⚙️ Actuation & Drive"]
        KINEMATICS["Mecanum Kinematics Controller"]
        JETSON["NVIDIA Jetson Nano / Motor Drivers"]
        MOTORS["4x Mecanum Wheel Motors"]

        LOCAL_PLAN -->|/cmd_vel| KINEMATICS
        JOY -->|/cmd_vel| KINEMATICS
        KINEMATICS --> JETSON
        JETSON --> MOTORS
    end
```

---

## 📂 Repository Structure

```
mecanum-4-wheel-ros2/
├── models/                      # Gazebo world assets, 3D meshes & environment models
├── simulation/                  # ROS 2 Simulation Stack
│   ├── iron_x/                  # Robot description (URDF/Xacro), Gazebo world & robot spawners
│   ├── nav2/                    # Nav2 2D/3D costmap params & waypoint navigation launch files
│   ├── point_cloud_perception/  # RTAB-Map 3D visual SLAM & PCL filter pipelines
│   ├── joy_stick/               # Teleop joy node integration (PS4)
│   └── test/                    # Kinematic and sensor unit tests
├── real/                        # Real Robot Deployment (NVIDIA Jetson Nano)
│   └── ironx_pro/               # Hardware drivers, live 2D/3D SLAM & real-world Nav2 launch scripts
└── README.md                    # Main documentation
```

---

## 🛠️ Prerequisites & Dependencies

### ROS 2 Packages Installation
Install required ROS 2 dependencies (replace `foxy` with `humble` or `jazzy` if using newer distributions):

```bash
sudo apt update
# Core simulation & robot description
sudo apt install -y \
  ros-foxy-joint-state-publisher-gui \
  ros-foxy-xacro \
  ros-foxy-robot-localization \
  ros-foxy-gazebo-ros-pkgs

# Navigation & SLAM
sudo apt install -y \
  ros-foxy-navigation2 \
  ros-foxy-nav2-bringup \
  ros-foxy-slam-toolbox

# 3D Perception & RTAB-Map
sudo apt install -y \
  ros-foxy-rtabmap* \
  ros-foxy-pcl-ros \
  pcl-tools

# Teleoperation
sudo apt install -y \
  ros-foxy-joy \
  ros-foxy-teleop-twist-joy
```

---

## 🚀 Quick Start Guide

### 1. Build the Workspace
```bash
# In your ROS 2 workspace root (e.g. ~/ros2_ws)
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 2. Gazebo Model Environment Setup
Ensure custom simulation models are available to Gazebo:
```bash
# Add simulation models to Gazebo model path
echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:'$(pwd)'/src/mecanum-4-wheel-ros2/models' >> ~/.bashrc
source ~/.bashrc
```

### 3. Launch Simulation & Robot Spawner
```bash
# Spawn robot in Gazebo world with RViz visualization
ros2 launch iron_x 2d_robot.launch.py
```

### 4. Run 2D SLAM Mapping
```bash
# In a new terminal, launch SLAM Toolbox for map building
ros2 launch iron_x slam.launch.py
# Or drive around using joystick teleop:
ros2 launch joy_stick joystick.launch.py
```

### 5. Autonomous Navigation with Nav2
```bash
# 2D Goal Pose Navigation
ros2 launch nav2 2d_pose.launch.py

# Or run automated multi-waypoint patrol sequence
ros2 launch nav2 2d_waypoint.launch.py
```

### 6. Real Robot Hardware Execution (Jetson Nano)
```bash
# On the physical robot:
ros2 launch ironx_pro 2d_nav.launch.py
# For 3D RTAB-Map autonomous waypoint navigation:
ros2 launch ironx_pro 3d_waypoint.launch.py
```

---

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](./LICENSE) file for details.
