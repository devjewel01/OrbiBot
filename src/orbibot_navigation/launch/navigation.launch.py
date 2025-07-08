#!/usr/bin/env python3
"""
OrbiBot Navigation Launch
Mode-specific navigation launching: SLAM, localization, or mapping only
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EqualsSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate navigation launch description"""
    
    # Package directories
    nav_pkg_dir = get_package_share_directory('orbibot_navigation')
    
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
    
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value=os.path.join(nav_pkg_dir, 'maps', 'map.yaml'),
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
    map_file = LaunchConfiguration('map_file')
    log_level = LaunchConfiguration('log_level')
    
    # Configuration files
    nav2_params_file = os.path.join(nav_pkg_dir, 'config', 'orbibot_nav2_params.yaml')
    slam_params_file = os.path.join(nav_pkg_dir, 'config', 'orbibot_slam_params.yaml')
    
    # SLAM Mode Launch
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('orbibot_navigation'),
                'launch',
                'slam.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file,
            'log_level': log_level,
        }.items(),
        condition=IfCondition(EqualsSubstitution(mode, 'slam'))
    )
    
    # Localization Mode Launch (Nav2 with AMCL)
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'map': map_file,
            'autostart': 'true',
            'use_composition': 'true',
            'use_respawn': 'false',
        }.items(),
        condition=IfCondition(EqualsSubstitution(mode, 'localization'))
    )
    
    # Navigation only (for SLAM mode)
    navigation_only_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'autostart': 'true',
            'use_composition': 'true',
            'use_respawn': 'false',
        }.items(),
        condition=IfCondition(EqualsSubstitution(mode, 'slam'))
    )
    
    # Mapping only mode (SLAM without navigation)
    mapping_only_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('orbibot_navigation'),
                'launch',
                'slam.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file,
            'log_level': log_level,
            'navigation': 'false',
        }.items(),
        condition=IfCondition(EqualsSubstitution(mode, 'mapping_only'))
    )
    
    return LaunchDescription([
        # Launch arguments
        mode_arg,
        use_sim_time_arg,
        map_file_arg,
        log_level_arg,
        
        # Mode-specific launches
        slam_launch,
        navigation_only_launch,
        localization_launch,
        mapping_only_launch,
    ])