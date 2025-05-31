Mobile Robot Simulation with Gazebo and RViz

This repository contains a ROS-based project for simulating a mobile robot moving in a circular path using Gazebo for physics simulation and RViz for visualization. The robot is modeled to follow a circular trajectory, and this project demonstrates how to set up the simulation environment, launch the robot in Gazebo, and visualize its movement in RViz.
Table of Contents

    Overview
    Prerequisites
    Installation
    Project Structure
    Usage
        Launching the Simulation
        Visualizing in RViz
    Customizing the Circular Motion
    Troubleshooting
    Contributing
    License

Overview

This project simulates a mobile robot (e.g., a differential drive robot) moving in a circular path. The simulation is built using:

    Gazebo: A 3D robot simulator for modeling the robot and its environment.
    RViz: A visualization tool for displaying sensor data and robot states.
    ROS: The Robot Operating System, used to manage communication between nodes, control the robot, and interface with Gazebo and RViz.

The robot follows a predefined circular trajectory by publishing velocity commands (linear and angular) to its /cmd_vel topic. The simulation includes a simple world in Gazebo and visualizes the robot's pose, trajectory, and sensor data (if applicable) in RViz.
Prerequisites

    Operating System: Ubuntu 20.04 (Focal) or 22.04 (Jammy) with ROS Noetic or ROS 2 Humble installed.
    ROS: Ensure ROS is installed. For ROS Noetic:
    bash

sudo apt install ros-noetic-desktop-full
For ROS 2 Humble:
bash
sudo apt install ros-humble-desktop
Gazebo: Installed with ROS or separately:
bash
sudo apt install gazebo11 ros-noetic-gazebo-ros-pkgs
or
bash
sudo apt install gazebo ros-humble-gazebo-ros-pkgs
RViz: Installed with ROS or separately:
bash
sudo apt install ros-noetic-rviz
or
bash

    sudo apt install ros-humble-rviz2
    Python: Python 3 for ROS scripts.
    Catkin Workspace (for ROS 1) or Colcon Workspace (for ROS 2).

Installation

    Set up a ROS Workspace: For ROS 1 (Noetic):
    bash

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone <repository-url>
cd ~/catkin_ws
catkin_make
source devel/setup.bash
For ROS 2 (Humble):
bash
mkdir -p ~/colcon_ws/src
cd ~/colcon_ws/src
git clone <repository-url>
cd ~/colcon_ws
colcon build
source install/setup.bash
Install Dependencies: Ensure all ROS dependencies are installed:
bash

    rosdep install --from-paths src --ignore-src -r -y
    Verify Gazebo and RViz: Run gazebo --version and rviz (or rviz2 for ROS 2) to confirm they are installed correctly.

Project Structure
text
mobile_robot/
├── launch/
│   ├── simulation.launch  # Launch file for Gazebo and RViz
├── scripts/
│   ├── circular_motion.py # Python script to control circular motion
├── urdf/
│   ├── robot.urdf         # URDF file defining the robot model
├── worlds/
│   ├── empty.world        # Gazebo world file
├── rviz/
│   ├── robot.rviz         # RViz configuration file
├── package.xml            # ROS package metadata
├── CMakeLists.txt         # Build configuration for ROS
└── README.md              # This file

    launch/: Contains the ROS launch file to start Gazebo, RViz, and the robot controller.
    scripts/: Python script to publish velocity commands for circular motion.
    urdf/: URDF file describing the robot's physical model (e.g., differential drive robot).
    worlds/: Gazebo world file defining the simulation environment.
    rviz/: RViz configuration for visualizing the robot's state.

Usage
Launching the Simulation

    Start the Simulation:
    For ROS 1:
    bash

source ~/catkin_ws/devel/setup.bash
roslaunch mobile_robot simulation.launch

For ROS 2:
bash

    source ~/colcon_ws/install/setup.bash
    ros2 launch mobile_robot simulation.launch

    This command:
        Launches Gazebo with the empty.world environment.
        Spawns the robot model defined in robot.urdf.
        Starts RViz with the robot.rviz configuration.
        Runs the circular_motion.py script to control the robot.

    What to Expect:
        Gazebo will display the robot moving in a circular path in a 3D environment.
        RViz will show the robot's pose, trajectory, and any sensor data (e.g., laser scans, if included).

Visualizing in RViz

    RViz Configuration:
        The robot.rviz file is preconfigured to display:
            Robot model (based on URDF).
            TF (transform) frames for the robot's position and orientation.
            Path topic (/robot_path) showing the robot's trajectory.
        To manually open RViz: For ROS 1:
        bash

rosrun rviz rviz -d $(rospack find mobile_robot)/rviz/robot.rviz
For ROS 2:
bash

        ros2 run rviz2 rviz2 -d $(ros2 pkg prefix mobile_robot)/share/mobile_robot/rviz/robot.rviz
    Topics to Visualize:
        /cmd_vel: Velocity commands sent to the robot.
        /odom: Odometry data for the robot's position and orientation.
        /robot_path: The robot's circular trajectory (published by the control script).

Customizing the Circular Motion

The circular_motion.py script publishes velocity commands to /cmd_vel to achieve circular motion. To modify the radius or speed:

    Open scripts/circular_motion.py.
    Adjust the parameters:
    python

linear_velocity = 0.5  # m/s
angular_velocity = 0.5  # rad/s

    Radius of the circular path is determined by linear_velocity / angular_velocity.
    Increase linear_velocity for a faster circular motion.
    Adjust angular_velocity to change the radius (smaller angular velocity = larger radius).

Re-run the simulation after modifying the script:
bash

    roslaunch mobile_robot simulation.launch

Troubleshooting

    Gazebo Fails to Load:
        Ensure Gazebo is installed (gazebo --version).
        Check for missing dependencies: rosdep install --from-paths src --ignore-src -r -y.
    Robot Not Moving:
        Verify the circular_motion.py script is running.
        Check if /cmd_vel topic is receiving messages: rostopic echo /cmd_vel (ROS 1) or ros2 topic echo /cmd_vel (ROS 2).
    RViz Shows No Data:
        Ensure the correct topics are subscribed in RViz.
        Check TF frames: rosrun tf view_frames (ROS 1) or ros2 run tf2_tools view_frames (ROS 2).
    Errors with URDF:
        Validate the URDF file: check_urdf robot.urdf.

Contributing

Contributions are welcome! To contribute:

    Fork the repository.
    Create a new branch: git checkout -b feature-name.
    Make changes and commit: git commit -m "Add feature".
    Push to the branch: git push origin feature-name.
    Open a pull request.

Please ensure your code follows ROS best practices and includes documentation.
License

This project is licensed under the MIT License. See the LICENSE file for details.
