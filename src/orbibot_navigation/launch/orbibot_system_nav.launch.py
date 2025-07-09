#!/usr/bin/env python3
"""
OrbiBot Complete System Navigation Launch
Integrates robot description, hardware, control, and navigation stack
Author: Claude Code Assistant
Updated: 2025-07-09
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate complete system navigation launch description"""
    
    # Package directories
    nav_pkg_dir = get_package_share_directory('orbibot_navigation')
    desc_pkg_dir = get_package_share_directory('orbibot_description')
    
    # Launch arguments
    declare_nav_mode_arg = DeclareLaunchArgument(
        'nav_mode',
        default_value='slam',
        description='Navigation mode: slam, localization, or mapping_only'
    )
    
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    declare_start_hardware_arg = DeclareLaunchArgument(
        'start_hardware',
        default_value='true',
        description='Start hardware interface'
    )
    
    declare_start_control_arg = DeclareLaunchArgument(
        'start_control',
        default_value='true',
        description='Start control manager'
    )
    
    declare_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz'
    )
    
    declare_params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(nav_pkg_dir, 'config', 'orbibot_nav2_params.yaml'),
        description='Navigation parameters file'
    )
    
    # Launch configuration variables
    nav_mode = LaunchConfiguration('nav_mode')
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_hardware = LaunchConfiguration('start_hardware')
    start_control = LaunchConfiguration('start_control')
    rviz = LaunchConfiguration('rviz')
    params_file = LaunchConfiguration('params_file')
    
    # Robot Description
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('orbibot_description'),
                'launch',
                'robot_state.launch.py'
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
        }.items(),
        condition=IfCondition(start_hardware)
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
        }.items(),
        condition=IfCondition(start_control)
    )
    
    # Navigation Stack
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('orbibot_navigation'),
                'launch',
                'orbibot_navigation.launch.py'
            ])
        ]),
        launch_arguments={
            'mode': nav_mode,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'rviz': rviz,
        }.items()
    )
    
    return LaunchDescription([
        # Launch arguments
        declare_nav_mode_arg,
        declare_use_sim_time_arg,
        declare_start_hardware_arg,
        declare_start_control_arg,
        declare_rviz_arg,
        declare_params_file_arg,
        
        # Launch components
        robot_description_launch,
        hardware_launch,
        control_launch,
        navigation_launch,
    ])