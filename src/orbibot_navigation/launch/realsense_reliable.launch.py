#!/usr/bin/env python3
"""
Reliable RealSense D435 launch file optimized for Raspberry Pi 5
Handles common USB bandwidth and power issues
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate reliable RealSense launch description"""
    
    # Launch arguments
    serial_no_arg = DeclareLaunchArgument(
        'serial_no',
        default_value="'250122076325'",
        description='RealSense serial number'
    )
    
    enable_color_arg = DeclareLaunchArgument(
        'enable_color',
        default_value='true',
        description='Enable color stream'
    )
    
    enable_depth_arg = DeclareLaunchArgument(
        'enable_depth',
        default_value='true', 
        description='Enable depth stream'
    )
    
    # Launch configuration
    serial_no = LaunchConfiguration('serial_no')
    enable_color = LaunchConfiguration('enable_color')
    enable_depth = LaunchConfiguration('enable_depth')
    
    # RealSense Camera Node with optimized settings for Pi 5
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        parameters=[{
            # Device selection
            'serial_no': serial_no,
            'device_type': 'D435',
            
            # Stream enables
            'enable_color': enable_color,
            'enable_depth': enable_depth,
            'enable_infra1': False,
            'enable_infra2': False,
            'enable_fisheye': False,
            'enable_gyro': False,
            'enable_accel': False,
            
            # Optimized profiles for Pi 5 (reduced bandwidth)
            'depth_module.profile': '424x240x15',  # Lower resolution, lower FPS
            'rgb_camera.profile': '424x240x15',    # Matching resolution and FPS
            
            # Processing options
            'pointcloud.enable': True,
            'pointcloud.stream_filter': 2,         # Color stream
            'pointcloud.stream_index_filter': 0,
            'align_depth.enable': True,            # Align depth to color
            
            # Filters to improve data quality
            'decimation_filter.enable': False,
            'spatial_filter.enable': True,
            'temporal_filter.enable': True,
            'hole_filling_filter.enable': True,
            'disparity_filter.enable': False,
            
            # Frame IDs
            'base_frame_id': 'camera_link',
            'depth_frame_id': 'camera_depth_frame',
            'color_frame_id': 'camera_color_frame',
            'infra_frame_id': 'camera_infra_frame',
            
            # TF publishing
            'publish_tf': True,
            'tf_publish_rate': 10.0,  # Reduced TF rate
            
            # Performance optimizations for Pi 5
            'depth_module.exposure.1': 8500,
            'depth_module.gain.1': 16,
            'depth_module.enable_auto_exposure.1': True,
            
            # USB and timing
            'reconnect_timeout': 6.0,
            'wait_for_device_timeout': -1.0,
            
            # Quality settings
            'depth_module.emitter_enabled': True,
            'depth_module.laser_power': 150,       # Reduced laser power
            'depth_module.accuracy': 2,            # Medium accuracy
            'depth_module.motion_range': 9,        # Auto motion range
            'depth_module.filter_option': 2,       # Fill from left
            'depth_module.confidence_threshold': 15,
        }],
        output='screen',
        emulate_tty=True,
        respawn=True,
        respawn_delay=2.0,
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    return LaunchDescription([
        # Launch arguments
        serial_no_arg,
        enable_color_arg,
        enable_depth_arg,
        
        # Camera node
        realsense_node,
    ])