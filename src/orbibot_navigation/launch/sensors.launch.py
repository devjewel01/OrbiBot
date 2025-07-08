#!/usr/bin/env python3
"""
OrbiBot Sensor System Launch
Configurable sensor launching for different sensor combinations
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EqualsSubstitution, OrSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate sensor launch description"""
    
    # Launch arguments
    sensors_arg = DeclareLaunchArgument(
        'sensors',
        default_value='lidar',
        description='Sensor configuration: lidar, realsense, or both',
        choices=['lidar', 'realsense', 'both']
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level for all nodes'
    )
    
    # Launch configuration
    sensors = LaunchConfiguration('sensors')
    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level = LaunchConfiguration('log_level')
    
    # LIDAR Node
    lidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'serial_port': '/dev/lidar',
            'serial_baudrate': 115200,
            'frame_id': 'laser_frame',
            'inverted': False,
            'angle_compensate': True,
            'use_sim_time': use_sim_time,
        }],
        arguments=['--ros-args', '--log-level', log_level],
        condition=IfCondition(OrSubstitution(
            EqualsSubstitution(sensors, 'lidar'),
            EqualsSubstitution(sensors, 'both')
        ))
    )
    
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
            'use_sim_time': use_sim_time,
            'enable_depth': 'true',
            'enable_color': 'true',
            'enable_infra1': 'false',
            'enable_infra2': 'false',
            'depth_module.profile': '640x480x30',
            'rgb_camera.profile': '640x480x30',
            'align_depth.enable': 'true',
            'pointcloud.enable': 'true',
        }.items(),
        condition=IfCondition(OrSubstitution(
            EqualsSubstitution(sensors, 'realsense'),
            EqualsSubstitution(sensors, 'both')
        ))
    )
    
    # Depth to LaserScan converter (for RealSense navigation)
    depth_to_scan_node = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depth_to_scan',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'output_frame': 'camera_depth_frame',
            'range_min': 0.4,
            'range_max': 4.0,
            'scan_height': 100,
            'scan_time': 0.033,
        }],
        remappings=[
            ('depth', '/camera/depth/image_rect_raw'),
            ('depth_camera_info', '/camera/depth/camera_info'),
            ('scan', '/scan_from_depth'),
        ],
        arguments=['--ros-args', '--log-level', log_level],
        condition=IfCondition(OrSubstitution(
            EqualsSubstitution(sensors, 'realsense'),
            EqualsSubstitution(sensors, 'both')
        ))
    )
    
    return LaunchDescription([
        # Launch arguments
        sensors_arg,
        use_sim_time_arg,
        log_level_arg,
        
        # Sensor nodes
        lidar_node,
        realsense_launch,
        depth_to_scan_node,
    ])