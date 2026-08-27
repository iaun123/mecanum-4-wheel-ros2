# 🌐 Gazebo Simulation Models & Environments

This directory contains the robot URDF/mesh assets and 3D simulation environment models used for Gazebo physics simulation.

---

## 📂 Directory Contents

| Folder | Description |
| :--- | :--- |
| [`model robot`](./model%20robot) | URDF descriptions, 3D STL meshes (base chassis, 4 Mecanum wheels, LiDAR, RGB-D camera, IMU), RViz display configs, and Gazebo spawners. |
| [`models`](./models) | 300+ 3D environment assets (AWS RoboMaker residential items, office furniture, obstacles, warehouse pallets, and custom worlds). |

---

## ⚙️ Gazebo Model Path Setup

To allow Gazebo to find and render these models properly, configure your environment using one of the following methods:

### Method 1: Export `GAZEBO_MODEL_PATH` in `.bashrc` (Recommended)

Add the path to this `models` directory to your `~/.bashrc`:

```bash
# Add models directory to Gazebo model path
echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:'$(pwd) >> ~/.bashrc
source ~/.bashrc
```

Or manually add this line to the end of `~/.bashrc`:
```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/path/to/mecanum-4-wheel-ros2/models:/path/to/mecanum-4-wheel-ros2/models/models
```

---

### Method 2: Link to Default `~/.gazebo/models` Directory

Alternatively, create a symbolic link directly inside your local `.gazebo` cache:

```bash
mkdir -p ~/.gazebo/models
ln -sf $(pwd)/models/* ~/.gazebo/models/
ln -sf "$(pwd)/model robot" ~/.gazebo/models/
```

---

## 🧪 Verifying Model Availability

Verify that Gazebo can detect the simulation models:

```bash
gazebo
```
In the Gazebo GUI, navigate to the **Insert** tab on the left panel to browse all loaded simulation models and spawn them into your world.
