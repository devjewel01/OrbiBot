#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory
    pkg_orbibot_sensors = get_package_share_directory('orbibot_sensors')
    
    # Declare launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/lidar',
        description='Serial port for RPLIDAR'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='laser',
        description='Frame ID for laser scan'
    )
    
    scan_mode_arg = DeclareLaunchArgument(
        'scan_mode',
        default_value='Standard',
        description='Scan mode for RPLIDAR'
    )
    
    # Parameter file path
    rplidar_params_file = os.path.join(
        pkg_orbibot_sensors,
        'config',
        'rplidar_params.yaml'
    )
    
    # RPLIDAR node
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_node',
        parameters=[
            rplidar_params_file,
            {
                'serial_port': LaunchConfiguration('serial_port'),
                'frame_id': LaunchConfiguration('frame_id'),
                'scan_mode': LaunchConfiguration('scan_mode'),
                'use_sim_time': False,
            }
        ],
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        remappings=[
            ('scan', '/scan'),
        ]
    )
    
    # Static transform publisher for laser frame
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_tf_broadcaster',
        arguments=['0', '0', '0.1', '0', '3.14159', '3.14159', 'base_link', 'laser'],
        output='screen'
    )
    
    # Log info
    log_info = LogInfo(
        msg='Starting RPLIDAR with parameters from: ' + rplidar_params_file
    )
    
    return LaunchDescription([
        serial_port_arg,
        frame_id_arg,
        scan_mode_arg,
        log_info,
        rplidar_node,
        static_transform_publisher,
    ])