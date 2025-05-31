#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    # Path to the empty Gazebo world
    gazebo_world = '/usr/share/gazebo-11/worlds/empty.world'

    # Start Gazebo server with ROS 2 integration plugins
    gazebo_server = ExecuteProcess(
        cmd=[
            'gazebo', '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            gazebo_world
        ],
        output='screen'
    )

    # Start Gazebo client for visualization
    gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    # Spawn TurtleBot3 robot at the origin
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-database', f'turtlebot3_{os.environ.get("TURTLEBOT3_MODEL", "burger")}',
            '-entity', 'robot',
            '-x', '0', '-y', '0', '-z', '0'
        ],
        output='screen'
    )

    # Launch the circular motion controller node
    motion_controller = Node(
        package='turtlebot3_circle',
        executable='circle_navigator',
        name='motion_controller',
        output='screen',
        arguments=['0.7', '0.1']
    )

    return LaunchDescription([
        gazebo_server,
        gazebo_client,
        spawn_robot,
        motion_controller,
    ])