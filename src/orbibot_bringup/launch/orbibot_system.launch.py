#!/usr/bin/env python3
"""
OrbiBot Complete System Launch
Integrates all robot components: hardware, control, sensors, localization, and navigation
Author: Claude Code Assistant
Created: 2025-07-14
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
    """Generate complete system launch description"""
    
    # Package directories
    bringup_pkg_dir = get_package_share_directory('orbibot_bringup')
    
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
    
    declare_start_sensors_arg = DeclareLaunchArgument(
        'start_sensors',
        default_value='true',
        description='Start sensors (LIDAR and camera)'
    )
    
    declare_start_localization_arg = DeclareLaunchArgument(
        'start_localization',
        default_value='true',
        description='Start enhanced localization'
    )
    
    declare_start_navigation_arg = DeclareLaunchArgument(
        'start_navigation',
        default_value='true',
        description='Start navigation stack'
    )
    
    declare_start_webui_arg = DeclareLaunchArgument(
        'start_webui',
        default_value='false',
        description='Start web monitoring interface'
    )
    
    declare_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz'
    )
    
    # Launch configuration variables
    nav_mode = LaunchConfiguration('nav_mode')
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_hardware = LaunchConfiguration('start_hardware')
    start_control = LaunchConfiguration('start_control')
    start_sensors = LaunchConfiguration('start_sensors')
    start_localization = LaunchConfiguration('start_localization')
    start_navigation = LaunchConfiguration('start_navigation')
    start_webui = LaunchConfiguration('start_webui')
    rviz = LaunchConfiguration('rviz')
    
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
    
    # Sensors
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('orbibot_sensors'),
                'launch',
                'sensors.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(start_sensors)
    )
    
    # Enhanced Localization
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('orbibot_localization'),
                'launch',
                'localization.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(start_localization)
    )
    
    # Navigation Stack
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('orbibot_navigation'),
                'launch',
                'navigation.launch.py'
            ])
        ]),
        launch_arguments={
            'mode': nav_mode,
            'use_sim_time': use_sim_time,
            'rviz': rviz,
        }.items(),
        condition=IfCondition(start_navigation)
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
        declare_nav_mode_arg,
        declare_use_sim_time_arg,
        declare_start_hardware_arg,
        declare_start_control_arg,
        declare_start_sensors_arg,
        declare_start_localization_arg,
        declare_start_navigation_arg,
        declare_start_webui_arg,
        declare_rviz_arg,
        
        # Launch components
        robot_description_launch,
        hardware_launch,
        control_launch,
        sensors_launch,
        localization_launch,
        navigation_launch,
        webui_launch,
    ])