#!/usr/bin/env python3
"""
OrbiBot Basic System Launch
Launches basic robot components: description, hardware, control, and teleop
Author: Claude Code Assistant
Created: 2025-07-14
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate basic system launch description"""
    
    # Launch arguments
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    declare_start_teleop_arg = DeclareLaunchArgument(
        'start_teleop',
        default_value='true',
        description='Start keyboard teleoperation'
    )
    
    declare_start_webui_arg = DeclareLaunchArgument(
        'start_webui',
        default_value='false',
        description='Start web monitoring interface'
    )
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_teleop = LaunchConfiguration('start_teleop')
    start_webui = LaunchConfiguration('start_webui')
    
    # Robot Description
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('orbibot_description'),
                'launch',
                'description.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )
    
    # Hardware Interface
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('orbibot_hardware'),
                'launch',
                'hardware.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )
    
    # Control Manager
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('orbibot_control'),
                'launch',
                'control.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )
    
    # Keyboard Teleoperation
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('orbibot_teleop'),
                'launch',
                'keyboard_teleop.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(start_teleop)
    )
    
    # Web UI
    webui_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('orbibot_webui'),
                'launch',
                'webui.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(start_webui)
    )
    
    return LaunchDescription([
        # Launch arguments
        declare_use_sim_time_arg,
        declare_start_teleop_arg,
        declare_start_webui_arg,
        
        # Launch components
        robot_description_launch,
        hardware_launch,
        control_launch,
        teleop_launch,
        webui_launch,
    ])