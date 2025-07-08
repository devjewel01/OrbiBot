#!/usr/bin/env python3
"""
Complete OrbiBot Navigation System Launch
Main entry point for all navigation modes
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
    """Generate complete OrbiBot navigation launch description"""
    
    # Launch arguments
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='slam',
        description='Navigation mode: slam, localization, or mapping_only',
        choices=['slam', 'localization', 'mapping_only']
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    hardware_arg = DeclareLaunchArgument(
        'hardware',
        default_value='True',
        description='Launch hardware nodes (robot_state_publisher, hardware, control)'
    )
    
    sensors_arg = DeclareLaunchArgument(
        'sensors',
        default_value='lidar',
        description='Sensor configuration: lidar, realsense, or both',
        choices=['lidar', 'realsense', 'both']
    )
    
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='True',
        description='Launch RViz visualization'
    )
    
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value='',
        description='Path to map file for localization mode'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level for all nodes'
    )
    
    # Launch configuration
    mode = LaunchConfiguration('mode')
    use_sim_time = LaunchConfiguration('use_sim_time')
    hardware = LaunchConfiguration('hardware')
    sensors = LaunchConfiguration('sensors')
    rviz = LaunchConfiguration('rviz')
    map_file = LaunchConfiguration('map_file')
    log_level = LaunchConfiguration('log_level')
    
    # Hardware System Launch
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('orbibot_navigation'),
                'launch',
                'hardware.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'log_level': log_level,
        }.items(),
        condition=IfCondition(hardware)
    )
    
    # Sensor Launch
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('orbibot_navigation'),
                'launch',
                'sensors.launch.py'
            ])
        ]),
        launch_arguments={
            'sensors': sensors,
            'use_sim_time': use_sim_time,
            'log_level': log_level,
        }.items()
    )
    
    # Navigation Launch (mode-specific)
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('orbibot_navigation'),
                'launch',
                'navigation.launch.py'
            ])
        ]),
        launch_arguments={
            'mode': mode,
            'use_sim_time': use_sim_time,
            'map_file': map_file,
            'log_level': log_level,
        }.items()
    )
    
    # RViz Launch
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('orbibot_navigation'),
                'launch',
                'rviz.launch.py'
            ])
        ]),
        launch_arguments={
            'mode': mode,
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(rviz)
    )
    
    return LaunchDescription([
        # Launch arguments
        mode_arg,
        use_sim_time_arg,
        hardware_arg,
        sensors_arg,
        rviz_arg,
        map_file_arg,
        log_level_arg,
        
        # Launch files
        hardware_launch,
        sensors_launch,
        navigation_launch,
        rviz_launch,
    ])