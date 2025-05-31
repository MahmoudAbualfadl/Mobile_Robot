# 🤖 Mobile Robot Simulation with Gazebo and RViz

[![ROS Version](https://img.shields.io/badge/ROS-Noetic%20%7C%20Humble-blue)](http://wiki.ros.org/)
[![License](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Gazebo](https://img.shields.io/badge/Gazebo-11-orange)](http://gazebosim.org/)
[![Python](https://img.shields.io/badge/Python-3.6%2B-green)](https://www.python.org/)

> A comprehensive ROS-based project for simulating a mobile robot following circular trajectories using Gazebo physics simulation and RViz visualization.

![Robot Simulation Demo](https://github.com/user-attachments/assets/a99f2f26-03ef-470b-b788-d869fbe0a886)

---

## 📋 Table of Contents

- [🎯 Overview](#-overview)
- [⚡ Features](#-features)
- [🔧 Prerequisites](#-prerequisites)
- [🚀 Quick Start](#-quick-start)
- [📁 Project Structure](#-project-structure)
- [🎮 Usage](#-usage)
- [⚙️ Customization](#️-customization)
- [🐛 Troubleshooting](#-troubleshooting)
- [🤝 Contributing](#-contributing)
- [📄 License](#-license)

---

## 🎯 Overview

This project demonstrates a **mobile robot simulation** that moves in precise circular patterns. Built on the Robot Operating System (ROS), it combines the power of Gazebo's physics engine with RViz's visualization capabilities to create an educational and development-friendly robotics environment.

### Key Technologies
- **🏗️ Gazebo**: 3D physics simulation environment
- **👁️ RViz**: Real-time robot state visualization
- **🔗 ROS**: Communication framework and node management
- **🐍 Python**: Robot control scripting

---

## ⚡ Features

- ✅ **Realistic Physics**: Full Gazebo integration with collision detection
- ✅ **Real-time Visualization**: Live robot tracking in RViz
- ✅ **Customizable Motion**: Adjustable circular path parameters
- ✅ **Cross-platform**: Support for ROS Noetic and ROS 2 Humble
- ✅ **Modular Design**: Easy to extend and modify
- ✅ **Educational**: Perfect for learning robotics concepts

---

## 🔧 Prerequisites

### System Requirements
| Component | Version | Installation |
|-----------|---------|--------------|
| **Ubuntu** | 20.04 / 22.04 | - |
| **ROS** | Noetic / Humble | `sudo apt install ros-noetic-desktop-full` |
| **Gazebo** | 11+ | `sudo apt install gazebo11 ros-noetic-gazebo-ros-pkgs` |
| **RViz** | Latest | `sudo apt install ros-noetic-rviz` |
| **Python** | 3.6+ | Built-in with Ubuntu |

### For ROS 2 Users
```bash
# ROS 2 Humble
sudo apt install ros-humble-desktop
sudo apt install gazebo ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-rviz2
```

---

## 🚀 Quick Start

### 1️⃣ Clone and Build

<details>
<summary><b>ROS 1 (Noetic)</b></summary>

```bash
# Create workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# Clone repository
git clone <repository-url>

# Build project
cd ~/catkin_ws
catkin_make

# Source environment
source devel/setup.bash
```
</details>

<details>
<summary><b>ROS 2 (Humble)</b></summary>

```bash
# Create workspace
mkdir -p ~/colcon_ws/src
cd ~/colcon_ws/src

# Clone repository
git clone <repository-url>

# Build project
cd ~/colcon_ws
colcon build

# Source environment
source install/setup.bash
```
</details>

### 2️⃣ Install Dependencies
```bash
rosdep install --from-paths src --ignore-src -r -y
```

### 3️⃣ Launch Simulation
```bash
# ROS 1
roslaunch mobile_robot simulation.launch

# ROS 2
ros2 launch mobile_robot simulation.launch
```

🎉 **That's it!** Your robot should now be moving in circles in Gazebo and visualized in RViz.

---

## 📁 Project Structure

```
mobile_robot/
├── 🚀 launch/
│   └── simulation.launch      # Main launch configuration
├── 📜 scripts/
│   └── circular_motion.py     # Robot control logic
├── 🤖 urdf/
│   └── robot.urdf            # Robot model definition
├── 🌍 worlds/
│   └── empty.world           # Simulation environment
├── 👁️ rviz/
│   └── robot.rviz            # Visualization settings
├── 📦 package.xml            # Package metadata
├── 🔨 CMakeLists.txt         # Build configuration
└── 📖 README.md              # This documentation
```

---

## 🎮 Usage

### Starting the Simulation

The simulation launches three main components simultaneously:

| Component | Purpose | Window |
|-----------|---------|---------|
| **Gazebo** | Physics simulation | 3D environment with robot |
| **RViz** | Data visualization | Robot state and trajectory |
| **Control Node** | Motion commands | Background process |

### Key Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands |
| `/odom` | `nav_msgs/Odometry` | Robot position data |
| `/robot_path` | `nav_msgs/Path` | Trajectory visualization |

### Manual RViz Launch
```bash
# ROS 1
rosrun rviz rviz -d $(rospack find mobile_robot)/rviz/robot.rviz

# ROS 2
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix mobile_robot)/share/mobile_robot/rviz/robot.rviz
```

---

## ⚙️ Customization

### Modifying Circular Motion

Edit `scripts/circular_motion.py` to customize the robot's behavior:

```python
# Motion parameters
linear_velocity = 0.5   # Forward speed (m/s)
angular_velocity = 0.5  # Rotation speed (rad/s)

# Calculate radius
radius = linear_velocity / angular_velocity
```

### Parameter Effects

| Parameter | Effect | Example |
|-----------|--------|---------|
| ↑ `linear_velocity` | Faster movement | 0.8 m/s = quicker circles |
| ↓ `angular_velocity` | Larger radius | 0.2 rad/s = wider circles |
| Equal values | Unit radius | 0.5/0.5 = 1m radius |

### Quick Presets

```python
# Tight circles
linear_velocity = 0.3
angular_velocity = 0.6

# Wide circles  
linear_velocity = 0.8
angular_velocity = 0.2

# Slow and steady
linear_velocity = 0.2
angular_velocity = 0.2
```

---

## 🐛 Troubleshooting

<details>
<summary><b>🔴 Gazebo fails to start</b></summary>

**Symptoms**: Black screen or crash on launch

**Solutions**:
```bash
# Check Gazebo installation
gazebo --version

# Reinstall if needed
sudo apt remove gazebo*
sudo apt install gazebo11

# Check graphics drivers
glxinfo | grep OpenGL
```
</details>

<details>
<summary><b>🔴 Robot not moving</b></summary>

**Symptoms**: Robot spawns but remains stationary

**Solutions**:
```bash
# Check if control script is running
rosnode list | grep circular

# Monitor velocity commands
rostopic echo /cmd_vel

# Restart the motion node
rosrun mobile_robot circular_motion.py
```
</details>

<details>
<summary><b>🔴 RViz shows empty scene</b></summary>

**Symptoms**: No robot or data visible in RViz

**Solutions**:
```bash
# Check TF frames
rosrun tf view_frames

# Verify topics
rostopic list

# Reset RViz config
File → Open Config → robot.rviz
```
</details>

<details>
<summary><b>🔴 URDF parsing errors</b></summary>

**Symptoms**: Robot model fails to load

**Solutions**:
```bash
# Validate URDF syntax
check_urdf urdf/robot.urdf

# View URDF structure
urdf_to_graphiz urdf/robot.urdf
```
</details>

### 📞 Need More Help?

- 📚 Check the [ROS Wiki](http://wiki.ros.org/)
- 💬 Ask on [ROS Answers](https://answers.ros.org/)
- 🐛 Open an [Issue](../../issues) on GitHub

---

## 🤝 Contributing

We welcome contributions! Here's how to get started:

### 📝 Contribution Guidelines

- ✅ Follow ROS coding standards
- ✅ Add tests for new features
- ✅ Update documentation
- ✅ Use conventional commit messages

### 🎯 Areas for Contribution

- 🐛 Bug fixes
- 📚 Documentation improvements
- ✨ New robot models
- 🔧 Additional motion patterns
- 🎨 RViz configurations

---

## 📄 License

This project is licensed under the **MIT License** - see the [LICENSE](LICENSE) file for details.

---

<div align="center">

**⭐ Star this repo if you found it helpful!**

Made with ❤️ for the robotics community

[🔝 Back to top](#-mobile-robot-simulation-with-gazebo-and-rviz)

</div>
