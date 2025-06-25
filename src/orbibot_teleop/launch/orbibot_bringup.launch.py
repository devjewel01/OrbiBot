#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    teleop_type = LaunchConfiguration('teleop_type')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    declare_teleop_type = DeclareLaunchArgument(
        'teleop_type',
        default_value='simple',
        description='Type of teleop: keyboard or simple'
    )
    
    # Get package directories
    hardware_pkg = get_package_share_directory('orbibot_hardware')
    control_pkg = get_package_share_directory('orbibot_control')
    teleop_pkg = get_package_share_directory('orbibot_teleop')
    
    # Hardware launch
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(hardware_pkg, 'launch', 'hardware.launch.py')
        ]),
        launch_arguments=[
            ('use_sim_time', use_sim_time)
        ]
    )
    
    # Control launch
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(control_pkg, 'launch', 'control.launch.py')
        ]),
        launch_arguments=[
            ('use_sim_time', use_sim_time)
        ]
    )
    
    # Teleop launch
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(teleop_pkg, 'launch', 'teleop.launch.py')
        ]),
        launch_arguments=[
            ('use_sim_time', use_sim_time),
            ('teleop_type', teleop_type)
        ]
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_teleop_type,
        
        # Launch nodes in order
        hardware_launch,
        control_launch,
        teleop_launch,
    ])