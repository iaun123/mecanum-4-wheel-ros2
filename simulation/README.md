# 💻 ROS 2 Simulation Stack

This directory contains the simulation packages for testing the 4-wheel Mecanum robot in Gazebo and RViz2.

---

## 📦 Packages Overview

| Package | Description |
| :--- | :--- |
| [`iron_x`](./iron_x) | Robot description (URDF/Xacro), Gazebo world spawner, and 2D/3D simulation launch files. |
| [`nav2`](./nav2) | Nav2 configuration, 2D goal pose navigation, and 2D/3D automated waypoint followers. |
| [`point_cloud_perception`](./point_cloud_perception) | RTAB-Map 3D visual SLAM and Point Cloud (PCL) processing pipeline. |
| [`joy_stick`](./joy_stick) | PS4 / DualShock joystick teleoperation node and velocity mappings. |
| [`test`](./test) | Kinematics calculation tests and simulation validation scripts. |

---

## 🚀 Quick Launch

```bash
# Launch robot in Gazebo simulation
ros2 launch iron_x 2d_robot.launch.py

# Run joystick control
ros2 launch joy_stick joystick.launch.py

# Launch Nav2 waypoint navigation
ros2 launch nav2 2d_waypoint.launch.py
```
