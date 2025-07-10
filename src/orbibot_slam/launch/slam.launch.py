#!/usr/bin/env python3
"""
OrbiBot SLAM Launch
SLAM mapping using slam_toolbox
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate SLAM launch description"""
    
    # Package directories
    slam_pkg_dir = get_package_share_directory('orbibot_slam')
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    slam_params_file_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(slam_pkg_dir, 'config', 'slam_params.yaml'),
        description='Path to SLAM parameters file'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level for SLAM node'
    )
    
    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    log_level = LaunchConfiguration('log_level')
    
    # SLAM Toolbox Node
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=[
            ('/scan', '/scan'),
            ('/map', '/map'),
            ('/odom', '/odom'),
        ]
    )
    
    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        slam_params_file_arg,
        log_level_arg,
        
        # SLAM node
        slam_toolbox_node,
    ])