#!/usr/bin/env python3
"""
OrbiBot Navigation Launch - Nav2 Stack Only
Simple launch file that only starts the Nav2 navigation stack
Author: Claude Code Assistant
Updated: 2025-07-14
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate Nav2 stack launch description only"""
    
    # Package directories
    nav_pkg_dir = get_package_share_directory('orbibot_navigation')
    
    # Launch arguments
    declare_params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(nav_pkg_dir, 'config', 'nav2_params.yaml'),
        description='Full path to the Nav2 parameters file'
    )
    
    declare_map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value=os.path.join(nav_pkg_dir, 'maps', 'map.yaml'),
        description='Full path to map file'
    )
    
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    declare_autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )
    
    declare_use_composition_arg = DeclareLaunchArgument(
        'use_composition',
        default_value='False',
        description='Use composed bringup if True'
    )
    
    declare_use_respawn_arg = DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description='Whether to respawn if a node crashes'
    )
    
    # Launch configuration variables
    params_file = LaunchConfiguration('params_file')
    map_file = LaunchConfiguration('map_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    
    # Nav2 Stack Launch
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            ])
        ]),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'use_composition': use_composition,
            'use_respawn': use_respawn,
        }.items()
    )
    
    return LaunchDescription([
        # Launch arguments
        declare_params_file_arg,
        declare_map_file_arg,
        declare_use_sim_time_arg,
        declare_autostart_arg,
        declare_use_composition_arg,
        declare_use_respawn_arg,
        
        # Nav2 Stack
        nav2_bringup_launch,
    ])