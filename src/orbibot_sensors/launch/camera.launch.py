#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory
    pkg_orbibot_sensors = get_package_share_directory('orbibot_sensors')
    
    # Declare launch arguments
    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='camera',
        description='Camera name for RealSense'
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
    
    enable_pointcloud_arg = DeclareLaunchArgument(
        'enable_pointcloud',
        default_value='true',
        description='Enable pointcloud generation'
    )
    
    color_width_arg = DeclareLaunchArgument(
        'color_width',
        default_value='640',
        description='Color stream width'
    )
    
    color_height_arg = DeclareLaunchArgument(
        'color_height',
        default_value='480',
        description='Color stream height'
    )
    
    color_fps_arg = DeclareLaunchArgument(
        'color_fps',
        default_value='30',
        description='Color stream FPS'
    )
    
    # Parameter file path
    realsense_params_file = os.path.join(
        pkg_orbibot_sensors,
        'config',
        'realsense_camera_params.yaml'
    )
    
    # RealSense camera node
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense2_camera',
        parameters=[
            realsense_params_file,
            {
                'camera_name': LaunchConfiguration('camera_name'),
                'enable_color': LaunchConfiguration('enable_color'),
                'enable_depth': LaunchConfiguration('enable_depth'),
                'enable_pointcloud': LaunchConfiguration('enable_pointcloud'),
                'color_width': LaunchConfiguration('color_width'),
                'color_height': LaunchConfiguration('color_height'),
                'color_fps': LaunchConfiguration('color_fps'),
            }
        ],
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        remappings=[
            ('~/color/image_raw', '/camera/color/image_raw'),
            ('~/depth/image_rect_raw', '/camera/depth/image_rect_raw'),
            ('~/color/camera_info', '/camera/color/camera_info'),
            ('~/depth/camera_info', '/camera/depth/camera_info'),
            ('~/color/metadata', '/camera/color/metadata'),
            ('~/depth/metadata', '/camera/depth/metadata'),
            ('~/extrinsics/depth_to_color', '/camera/extrinsics/depth_to_color'),
            ('/clicked_point', '/clicked_point'),
        ]
    )
    
    # Static transform publisher for camera frame
    camera_tf_broadcaster = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf_broadcaster',
        arguments=['0.1', '0', '0.15', '0', '0', '0', 'base_link', 'camera_link'],
        output='screen'
    )
    
    # Static transform for camera optical frame (ROS optical frame convention)
    camera_optical_tf_broadcaster = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_optical_tf_broadcaster',
        arguments=['0', '0', '0', '-1.5708', '0', '-1.5708', 'camera_link', 'camera_color_optical_frame'],
        output='screen'
    )
    
    # Log info
    log_info = LogInfo(
        msg='Starting RealSense camera with parameters from: ' + realsense_params_file
    )
    
    return LaunchDescription([
        camera_name_arg,
        enable_color_arg,
        enable_depth_arg,
        enable_pointcloud_arg,
        color_width_arg,
        color_height_arg,
        color_fps_arg,
        log_info,
        realsense_node,
        camera_tf_broadcaster,
        camera_optical_tf_broadcaster,
    ])