#!/usr/bin/env python3
"""
Launch file for OrbiBot autonomous navigation using Nav2
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for navigation"""
    
    # Package directories
    nav_pkg_dir = get_package_share_directory('orbibot_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Configuration files
    nav2_params_file = os.path.join(nav_pkg_dir, 'config', 'nav2_params.yaml')
    
    # Map file (if available)
    map_yaml_file = os.path.join(nav_pkg_dir, 'maps', 'map.yaml')
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=nav2_params_file,
        description='Full path to the ROS2 parameters file for Nav2'
    )
    
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=map_yaml_file,
        description='Full path to map yaml file to load'
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
    
    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )
    
    use_composition_arg = DeclareLaunchArgument(
        'use_composition',
        default_value='true',
        description='Whether to use composed bringup'
    )
    
    use_localization_arg = DeclareLaunchArgument(
        'use_localization',
        default_value='false',
        description='Whether to use localization (AMCL) instead of SLAM'
    )
    
    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')\n    map_yaml_file = LaunchConfiguration('map')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_localization = LaunchConfiguration('use_localization')
    
    # Set environment variable for DDS
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')
    
    # Nav2 Bringup Launch
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            ])
        ]),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'use_composition': use_composition,
            'use_respawn': use_respawn,
        }.items(),
        condition=IfCondition(use_localization)
    )
    
    # Alternative: Nav2 Navigation Launch (without map server and AMCL for SLAM mode)
    nav2_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'use_composition': use_composition,
            'use_respawn': use_respawn,
        }.items(),
        condition=UnlessCondition(use_localization)
    )
    
    # Map Server (for localization mode)
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_yaml_file
        }],
        arguments=['--ros-args', '--log-level', log_level],
        condition=IfCondition(use_localization)
    )
    
    # AMCL (for localization mode)
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=[
            ('/scan', '/scan'),
            ('/odom', '/orbibot/odometry/filtered')
        ],
        condition=IfCondition(use_localization)
    )
    
    # Lifecycle Manager for localization nodes
    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': ['map_server', 'amcl']
        }],
        condition=IfCondition(use_localization)
    )
    
    return LaunchDescription([
        # Environment variables
        stdout_linebuf_envvar,
        
        # Launch arguments
        use_sim_time_arg,
        params_file_arg,
        map_arg,
        use_respawn_arg,
        log_level_arg,
        autostart_arg,
        use_composition_arg,
        use_localization_arg,
        
        # Conditional launches
        nav2_bringup_launch,           # Full bringup for localization mode
        nav2_navigation_launch,        # Navigation only for SLAM mode
        
        # Localization nodes (only in localization mode)
        map_server_node,
        amcl_node,
        lifecycle_manager_localization,
    ])