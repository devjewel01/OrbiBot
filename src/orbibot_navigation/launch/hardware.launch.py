#!/usr/bin/env python3
"""
OrbiBot Hardware System Launch
Launches robot description, hardware interface, and control nodes
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate hardware system launch description"""
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level for all nodes'
    )
    
    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level = LaunchConfiguration('log_level')
    
    # Robot Description Launch
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('orbibot_description'),
                'launch',
                'display.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )
    
    # Hardware Interface Launch
    hardware_interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('orbibot_hardware'),
                'launch',
                'hardware.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'log_level': log_level,
        }.items()
    )
    
    # Control System Launch
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
            'log_level': log_level,
        }.items()
    )
    
    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        log_level_arg,
        
        # Launch files
        robot_description_launch,
        hardware_interface_launch,
        control_launch,
    ])