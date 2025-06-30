#!/usr/bin/env python3
"""
Complete OrbiBot System Launch
Launches hardware interface + control manager
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directories
    hardware_pkg = get_package_share_directory('orbibot_hardware')
    control_pkg = get_package_share_directory('orbibot_control')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    # Hardware launch
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(hardware_pkg, 'launch', 'hardware.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # Control manager launch  
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(control_pkg, 'launch', 'control.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        hardware_launch,
        control_launch
    ])