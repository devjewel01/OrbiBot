#!/usr/bin/env python3
"""
OrbiBot Navigation Launch - Refactored and Cleaned Up
Main navigation launch file supporting multiple modes: SLAM, localization, mapping only
Author: Claude Code Assistant
Updated: 2025-07-08
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EqualsSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate the complete navigation launch description"""
    
    # Package directories
    nav_pkg_dir = get_package_share_directory('orbibot_navigation')
    
    # Launch arguments
    declare_mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='slam',
        description='Navigation mode: slam, localization, or mapping_only',
        choices=['slam', 'localization', 'mapping_only']
    )
    
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    declare_params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(nav_pkg_dir, 'config', 'orbibot_nav2_params.yaml'),
        description='Full path to the ROS2 parameters file'
    )
    
    declare_map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value=os.path.join(nav_pkg_dir, 'maps', 'map.yaml'),
        description='Full path to map file for localization mode'
    )
    
    declare_slam_params_file_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(nav_pkg_dir, 'config', 'orbibot_slam_params.yaml'),
        description='Full path to SLAM parameters file'
    )
    
    declare_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz if true'
    )
    
    declare_rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(nav_pkg_dir, 'config', 'orbibot_slam.rviz'),
        description='Full path to RViz config file'
    )
    
    # Launch configuration variables
    mode = LaunchConfiguration('mode')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    map_file = LaunchConfiguration('map_file')
    slam_params_file = LaunchConfiguration('slam_params_file')
    rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    
    # SLAM Mode Group
    slam_group = GroupAction([
        # SLAM Toolbox
        IncludeLaunchDescription(
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
            }.items()
        ),
        
        # Navigation Stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('nav2_bringup'),
                    'launch',
                    'navigation_launch.py'
                ])
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': params_file,
                'autostart': 'true',
                'use_composition': 'False',
                'use_respawn': 'False',
            }.items()
        ),
    ],
    condition=IfCondition(EqualsSubstitution(mode, 'slam'))
    )
    
    # Localization Mode Group (with pre-built map)
    localization_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('nav2_bringup'),
                    'launch',
                    'bringup_launch.py'
                ])
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': params_file,
                'map': map_file,
                'autostart': 'true',
                'use_composition': 'False',
                'use_respawn': 'False',
            }.items()
        ),
    ],
    condition=IfCondition(EqualsSubstitution(mode, 'localization'))
    )
    
    # Mapping Only Mode Group (SLAM without navigation)
    mapping_only_group = GroupAction([
        IncludeLaunchDescription(
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
            }.items()
        ),
    ],
    condition=IfCondition(EqualsSubstitution(mode, 'mapping_only'))
    )
    
    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(rviz)
    )
    
    return LaunchDescription([
        # Launch arguments
        declare_mode_arg,
        declare_use_sim_time_arg,
        declare_params_file_arg,
        declare_map_file_arg,
        declare_slam_params_file_arg,
        declare_rviz_arg,
        declare_rviz_config_arg,
        
        # Launch groups
        slam_group,
        localization_group,
        mapping_only_group,
        
        # Optional RViz
        rviz_node,
    ])