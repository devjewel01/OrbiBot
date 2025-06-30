#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch OrbiBot hardware interface
    """
    
    # Package directories
    pkg_orbibot_hardware = get_package_share_directory('orbibot_hardware')
    
    # Configuration files
    hardware_config = PathJoinSubstitution([
        pkg_orbibot_hardware, 'config', 'hardware_params.yaml'
    ])
    
    # Launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    declare_serial_port = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/motordriver',
        description='Serial port for motor driver'
    )
    
    declare_start_enabled = DeclareLaunchArgument(
        'motors_enabled',
        default_value='false',
        description='Start with motors enabled'
    )
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    serial_port = LaunchConfiguration('serial_port')
    motors_enabled = LaunchConfiguration('motors_enabled')
    
    # Hardware interface node
    hardware_node = Node(
        package='orbibot_hardware',
        executable='hardware_node',
        name='orbibot_hardware_node',
        output='screen',
        parameters=[
            hardware_config,
            {
                'use_sim_time': use_sim_time,
                'hardware.serial_port': serial_port,
            }
        ],
        remappings=[
            # Remap topics if needed
            ('/orbibot/wheel_speeds', '/orbibot/wheel_speeds'),
            ('/cmd_vel_raw', '/cmd_vel_raw'),
        ],
        condition=IfCondition('true')  # Always run unless sim mode
    )
    
    # Motor enable service call (delayed start) - only if requested
    motor_enable_cmd = TimerAction(
        period=3.0,  # Wait 3 seconds after hardware node starts
        actions=[
            Node(
                package='orbibot_hardware',
                executable='motor_enable_client',
                name='motor_enable_startup',
                parameters=[{'enable': motors_enabled}],
                condition=IfCondition(motors_enabled)
            )
        ]
    )
    
    return LaunchDescription([
        # Arguments
        declare_use_sim_time,
        declare_serial_port,
        declare_start_enabled,
        
        # Nodes
        hardware_node,
     
        # Delayed actions
        motor_enable_cmd,
    ])