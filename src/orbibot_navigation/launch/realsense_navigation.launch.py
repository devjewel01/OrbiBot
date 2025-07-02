#!/usr/bin/env python3
"""
RealSense-enhanced navigation launch file for OrbiBot
Integrates depth camera for improved autonomous driving
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate RealSense-enhanced navigation launch description"""
    
    # Package directories
    nav_pkg_dir = get_package_share_directory('orbibot_navigation')
    realsense_params_file = os.path.join(nav_pkg_dir, 'config', 'realsense_params.yaml')
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    enable_visual_odometry_arg = DeclareLaunchArgument(
        'enable_visual_odometry',
        default_value='true',
        description='Enable visual odometry'
    )
    
    enable_depth_obstacles_arg = DeclareLaunchArgument(
        'enable_depth_obstacles',
        default_value='true',
        description='Enable depth-based obstacle detection'
    )
    
    enable_visual_slam_arg = DeclareLaunchArgument(
        'enable_visual_slam',
        default_value='false',
        description='Enable visual SLAM (RTAB-Map)'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level'
    )
    
    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_visual_odometry = LaunchConfiguration('enable_visual_odometry')
    enable_depth_obstacles = LaunchConfiguration('enable_depth_obstacles')
    enable_visual_slam = LaunchConfiguration('enable_visual_slam')
    log_level = LaunchConfiguration('log_level')
    
    # RealSense Camera Launch
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py'
            ])
        ]),
        launch_arguments={
            'enable_color': 'true',
            'enable_depth': 'true',
            'depth_module.profile': '640x480x30',
            'rgb_camera.profile': '640x480x30',
            'pointcloud.enable': 'true',
            'align_depth.enable': 'true',
            'use_sim_time': use_sim_time,
        }.items()
    )
    
    # Depth Image to LaserScan Converter
    # Converts depth image to 2D laser scan for additional obstacle detection
    depth_to_scan_node = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depth_to_scan',
        parameters=[
            realsense_params_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('depth', '/camera/aligned_depth_to_color/image_raw'),
            ('depth_camera_info', '/camera/aligned_depth_to_color/camera_info'),
            ('scan', '/depth_scan')
        ],
        condition=IfCondition(enable_depth_obstacles),
        output='screen'
    )
    
    # Visual Odometry Node (RTAB-Map)
    visual_odometry_node = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='visual_odometry',
        parameters=[
            realsense_params_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('rgb/image', '/camera/color/image_raw'),
            ('depth/image', '/camera/aligned_depth_to_color/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('odom', '/visual_odometry/odom'),
            ('rgbd_image', '/visual_odometry/rgbd_image')
        ],
        arguments=['--ros-args', '--log-level', log_level],
        condition=IfCondition(enable_visual_odometry),
        output='screen'
    )
    
    # 3D Obstacle Detection Node
    obstacle_detector_node = Node(
        package='orbibot_navigation',
        executable='obstacle_detector',
        name='obstacle_detector_3d',
        parameters=[
            realsense_params_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('cloud_in', '/camera/depth/color/points'),
            ('obstacles', '/obstacles_3d'),
            ('obstacle_markers', '/obstacle_markers_3d')
        ],
        arguments=['--ros-args', '--log-level', log_level],
        condition=IfCondition(enable_depth_obstacles),
        output='screen'
    )
    
    # Visual SLAM Node (RTAB-Map)
    visual_slam_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='visual_slam',
        parameters=[
            realsense_params_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('rgb/image', '/camera/color/image_raw'),
            ('depth/image', '/camera/aligned_depth_to_color/image_raw'), 
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('odom', '/orbibot/odometry/filtered'),
            ('grid_map', '/visual_map')
        ],
        arguments=['--ros-args', '--log-level', log_level, '--delete_db_on_start'],
        condition=IfCondition(enable_visual_slam),
        output='screen'
    )
    
    # Enhanced Localization with Visual Odometry
    enhanced_localization_node = Node(
        package='orbibot_localization',
        executable='sensor_fusion',
        name='enhanced_sensor_fusion',
        parameters=[
            realsense_params_file,
            {
                'use_sim_time': use_sim_time,
                'enable_visual_odometry': enable_visual_odometry
            }
        ],
        remappings=[
            ('/visual_odometry', '/visual_odometry/odom'),
            ('/enhanced_odom', '/orbibot/odometry/enhanced')
        ],
        arguments=['--ros-args', '--log-level', log_level],
        condition=IfCondition(enable_visual_odometry),
        output='screen'
    )
    
    # Multi-sensor LaserScan merger (combines LIDAR + depth scan)
    scan_merger_node = Node(
        package='ira_laser_tools',
        executable='laserscan_multi_merger',
        name='scan_merger',
        parameters=[{
            'destination_frame': 'base_link',
            'cloud_destination_topic': '/merged_scan',
            'scan_destination_topic': '/scan_merged',
            'laserscan_topics': '/scan /depth_scan',
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0087,
            'scan_time': 0.033,
            'range_min': 0.1,
            'range_max': 8.0,
            'use_sim_time': use_sim_time
        }],
        condition=IfCondition(enable_depth_obstacles),
        output='screen'
    )
    
    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        enable_visual_odometry_arg,
        enable_depth_obstacles_arg,
        enable_visual_slam_arg,
        log_level_arg,
        
        # Camera and processing nodes
        realsense_launch,
        depth_to_scan_node,
        visual_odometry_node,
        obstacle_detector_node,
        visual_slam_node,
        enhanced_localization_node,
        # scan_merger_node,  # Requires ira_laser_tools package
    ])