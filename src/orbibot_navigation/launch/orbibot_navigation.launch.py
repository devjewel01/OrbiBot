#!/usr/bin/env python3
"""
Complete OrbiBot navigation system launch file
Includes hardware, localization, SLAM, and navigation
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate complete navigation launch description"""
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    slam_arg = DeclareLaunchArgument(
        'slam',
        default_value='true',
        description='Whether to run SLAM'
    )
    
    navigation_arg = DeclareLaunchArgument(
        'navigation',
        default_value='true',
        description='Whether to run navigation'
    )
    
    localization_arg = DeclareLaunchArgument(
        'localization',
        default_value='true',
        description='Whether to run enhanced localization'
    )
    
    hardware_arg = DeclareLaunchArgument(
        'hardware',
        default_value='true',
        description='Whether to launch hardware nodes'
    )
    
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Whether to start RViz'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level'
    )
    
    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam = LaunchConfiguration('slam')
    navigation = LaunchConfiguration('navigation')
    localization = LaunchConfiguration('localization')
    hardware = LaunchConfiguration('hardware')
    rviz = LaunchConfiguration('rviz')
    log_level = LaunchConfiguration('log_level')
    
    # Robot Description Launch
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('orbibot_description'),
                'launch',
                'robot_state.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(hardware)
    )
    
    # Hardware Launch
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('orbibot_hardware'),
                'launch',
                'hardware.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(hardware)
    )
    
    # Control Launch
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('orbibot_control'),
                'launch',
                'control.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(hardware)
    )
    
    # Enhanced Localization Launch
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('orbibot_localization'),
                'launch',
                'localization.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'log_level': log_level,
        }.items(),
        condition=IfCondition(localization)
    )
    
    # SLAM Launch
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('orbibot_navigation'),
                'launch',
                'slam.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'log_level': log_level,
        }.items(),
        condition=IfCondition(slam)
    )
    
    # Navigation Launch
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('orbibot_navigation'),
                'launch',
                'navigation.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_localization': UnlessCondition(slam),
            'log_level': log_level,
        }.items(),
        condition=IfCondition(navigation)
    )
    
    # RViz Launch
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'rviz_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(rviz)
    )
    
    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        slam_arg,
        navigation_arg,
        localization_arg,
        hardware_arg,
        rviz_arg,
        log_level_arg,
        
        # Launch files
        robot_description_launch,
        hardware_launch,
        control_launch,
        localization_launch,
        slam_launch,
        navigation_launch,
        rviz_launch,
    ])