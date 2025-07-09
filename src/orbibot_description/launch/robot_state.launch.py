#!/usr/bin/env python3

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
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    
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
            'use_sim_time': use_sim_time,
        }]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        robot_state_publisher,
    ])