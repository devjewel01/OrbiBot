#!/usr/bin/env python3
"""
Launch file for OrbiBot SLAM using slam_toolbox
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for SLAM"""
    
    # Package directories
    nav_pkg_dir = get_package_share_directory('orbibot_navigation')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    
    # Configuration files
    slam_params_file = os.path.join(nav_pkg_dir, 'config', 'slam_params.yaml')
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    slam_params_file_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=slam_params_file,
        description='Full path to the ROS2 parameters file for SLAM'
    )
    
    use_respawn_arg = DeclareLaunchArgument(
        'use_respawn',
        default_value='false',
        description='Whether to respawn if a node crashes'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='log level'
    )
    
    launch_lidar_arg = DeclareLaunchArgument(
        'launch_lidar',
        default_value='true',
        description='Whether to launch LIDAR node'
    )
    
    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    launch_lidar = LaunchConfiguration('launch_lidar')
    
    # SLAM Toolbox Node
    start_async_slam_toolbox_node = Node(
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        respawn=use_respawn,
        arguments=['--ros-args', '--log-level', log_level],
        remappings=[
            ('/scan', '/scan'),
            ('/map', '/map'),
            ('/odom', '/odom')
        ]
    )
    
    # Static transform from map to odom (identity for SLAM)
    # This will be overridden by the SLAM node
    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        condition=UnlessCondition('true')  # Disabled - SLAM toolbox handles this
    )
    
    # LIDAR Launch
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('orbibot_navigation'),
                'launch',
                'lidar.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(launch_lidar)
    )
    
    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        slam_params_file_arg,
        use_respawn_arg,
        log_level_arg,
        launch_lidar_arg,
        
        # Nodes
        start_async_slam_toolbox_node,
        # map_to_odom_tf,  # Disabled
        lidar_launch,
    ])