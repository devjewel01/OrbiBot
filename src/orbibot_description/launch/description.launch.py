#!/usr/bin/env python3

""" OrbiBot Run On Raspberry Pi 5 without gui using Ubuntu 24.04 Server & GUI Rviz See using Computer with same ROS domain Id-42 """

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """Generate robot state publisher launch description"""
    
    # Package directory
    pkg_dir = get_package_share_directory('orbibot_description')
    
    # URDF file path
    urdf_file = os.path.join(pkg_dir, 'urdf', 'orbibot.urdf.xacro')
    
    # Robot description
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
        }]
    )

    # Joint State Publisher node - publishes wheel joint states
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
    )
    
    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_node
    ])