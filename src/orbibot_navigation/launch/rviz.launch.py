#!/usr/bin/env python3
"""
OrbiBot RViz Launch
Mode-specific RViz configuration launching
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, EqualsSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate RViz launch description"""
    
    # Package directories
    nav_pkg_dir = get_package_share_directory('orbibot_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Launch arguments
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='slam',
        description='Navigation mode for RViz config selection',
        choices=['slam', 'localization', 'mapping_only']
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # Launch configuration
    mode = LaunchConfiguration('mode')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # RViz configuration files
    slam_rviz_config = os.path.join(nav_pkg_dir, 'config', 'orbibot_slam.rviz')
    nav_rviz_config = os.path.join(nav_pkg_dir, 'config', 'orbibot_navigation.rviz')
    
    # Default to nav2_bringup config if custom configs don't exist
    default_rviz_config = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    
    # RViz Node for SLAM mode
    rviz_slam_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', slam_rviz_config if os.path.exists(slam_rviz_config) else default_rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(EqualsSubstitution(mode, 'slam'))
    )
    
    # RViz Node for localization mode
    rviz_nav_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', nav_rviz_config if os.path.exists(nav_rviz_config) else default_rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(EqualsSubstitution(mode, 'localization'))
    )
    
    # RViz Node for mapping only mode
    rviz_mapping_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', slam_rviz_config if os.path.exists(slam_rviz_config) else default_rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(EqualsSubstitution(mode, 'mapping_only'))
    )
    
    return LaunchDescription([
        # Launch arguments
        mode_arg,
        use_sim_time_arg,
        
        # RViz nodes (mode-specific)
        rviz_slam_node,
        rviz_nav_node,
        rviz_mapping_node,
    ])