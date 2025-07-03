#!/usr/bin/env python3
"""
Launch file for RPLIDAR A1
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for RPLIDAR A1"""
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/lidar',
        description='Serial port for LIDAR'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='lidar_link',
        description='Frame ID for laser scan'
    )
    
    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time')
    serial_port = LaunchConfiguration('serial_port')
    frame_id = LaunchConfiguration('frame_id')
    
    # RPLIDAR Node
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_node',
        parameters=[{
            'serial_port': serial_port,
            'frame_id': frame_id,
            'inverted': False,
            'angle_compensate': True,
            'scan_frequency': 10.0,
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )
    
    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        serial_port_arg,
        frame_id_arg,
        
        # Nodes
        rplidar_node,
    ])