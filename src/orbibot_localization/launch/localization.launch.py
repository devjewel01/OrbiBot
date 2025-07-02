#!/usr/bin/env python3
"""
Launch file for OrbiBot localization system
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for localization system"""
    
    # Get package directory
    pkg_dir = get_package_share_directory('orbibot_localization')
    config_file = os.path.join(pkg_dir, 'config', 'localization_params.yaml')
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    enable_visual_arg = DeclareLaunchArgument(
        'enable_visual',
        default_value='false',
        description='Enable visual odometry'
    )
    
    enable_lidar_arg = DeclareLaunchArgument(
        'enable_lidar',
        default_value='false', 
        description='Enable lidar odometry'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level'
    )
    
    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_visual = LaunchConfiguration('enable_visual')
    enable_lidar = LaunchConfiguration('enable_lidar')
    log_level = LaunchConfiguration('log_level')
    
    # EKF Localization Node
    ekf_localization_node = Node(
        package='orbibot_localization',
        executable='ekf_localization',
        name='ekf_localization',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=[
            ('/odom', '/odom'),
            ('/imu/data', '/imu/data'),
        ]
    )
    
    # Sensor Fusion Node
    sensor_fusion_node = Node(
        package='orbibot_localization',
        executable='sensor_fusion',
        name='sensor_fusion',
        output='screen',
        parameters=[
            config_file,
            {
                'use_sim_time': use_sim_time,
                'enable_visual_odometry': enable_visual,
                'enable_lidar_odometry': enable_lidar
            }
        ],
        arguments=['--ros-args', '--log-level', log_level],
        condition=IfCondition('false')  # Disable for now, will enable when implemented
    )
    
    # Robot Localization (EKF) from robot_localization package
    # This provides an alternative/backup localization method
    robot_localization_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='robot_localization_ekf',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frequency': 30.0,
            'sensor_timeout': 0.1,
            'two_d_mode': True,
            'transform_time_offset': 0.0,
            'transform_timeout': 0.0,
            'print_diagnostics': True,
            'debug': False,
            
            # Frame configuration
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_link_frame': 'base_link',
            'world_frame': 'odom',
            
            # Odometry configuration
            'odom0': '/odom',
            'odom0_config': [True,  True,  False,   # x, y, z
                           False, False, True,     # roll, pitch, yaw
                           True,  True,  False,    # vx, vy, vz
                           False, False, True,     # vroll, vpitch, vyaw
                           False, False, False],   # ax, ay, az
            'odom0_queue_size': 10,
            'odom0_nodelay': False,
            'odom0_differential': False,
            'odom0_relative': False,
            
            # IMU configuration
            'imu0': '/imu/data',
            'imu0_config': [False, False, False,    # x, y, z
                          False, False, True,      # roll, pitch, yaw
                          False, False, False,     # vx, vy, vz
                          False, False, True,      # vroll, vpitch, vyaw
                          False, False, False],    # ax, ay, az
            'imu0_queue_size': 10,
            'imu0_nodelay': False,
            'imu0_differential': False,
            'imu0_relative': False,
            'imu0_remove_gravitational_acceleration': True,
            
            # Process noise covariance
            'process_noise_covariance': [0.05, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                                       0.0,  0.05, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                                       0.0,  0.0,  0.06, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                                       0.0,  0.0,  0.0,  0.03, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                                       0.0,  0.0,  0.0,  0.0,  0.03, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                                       0.0,  0.0,  0.0,  0.0,  0.0,  0.06, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                                       0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.025, 0.0, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                                       0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.025, 0.0, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                                       0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.04, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                                       0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.01, 0.0,  0.0,  0.0,  0.0,  0.0,
                                       0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.01, 0.0,  0.0,  0.0,  0.0,
                                       0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.02, 0.0,  0.0,  0.0,
                                       0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.01, 0.0,  0.0,
                                       0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.01, 0.0,
                                       0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.015],
            
            # Initial estimate covariance
            'initial_estimate_covariance': [1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                                          0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                                          0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                                          0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                                          0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                                          0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                                          0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                                          0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                                          0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                                          0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,
                                          0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,
                                          0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,
                                          0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,
                                          0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,
                                          0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9]
        }],
        condition=IfCondition('false'),  # Disabled by default, use custom EKF
        remappings=[
            ('/odometry/filtered', '/robot_localization/odometry/filtered')
        ]
    )
    
    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        enable_visual_arg,
        enable_lidar_arg,
        log_level_arg,
        
        # Nodes
        ekf_localization_node,
        # sensor_fusion_node,  # Disabled for now
        # robot_localization_ekf,  # Alternative EKF (disabled by default)
    ])