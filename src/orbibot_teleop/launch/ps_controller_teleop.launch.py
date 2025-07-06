#!/usr/bin/env python3
"""
PlayStation Controller Teleop Launch File for OrbiBot
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for PlayStation controller teleop"""
    
    # Launch arguments
    max_linear_speed_arg = DeclareLaunchArgument(
        'max_linear_speed',
        default_value='1.0',
        description='Maximum linear speed (m/s)'
    )
    
    max_angular_speed_arg = DeclareLaunchArgument(
        'max_angular_speed', 
        default_value='1.5',
        description='Maximum angular speed (rad/s)'
    )
    
    deadzone_arg = DeclareLaunchArgument(
        'deadzone',
        default_value='0.1',
        description='Analog stick deadzone (0.0-1.0)'
    )
    
    turbo_multiplier_arg = DeclareLaunchArgument(
        'turbo_multiplier',
        default_value='2.0',
        description='Speed multiplier for turbo mode'
    )
    
    precision_multiplier_arg = DeclareLaunchArgument(
        'precision_multiplier',
        default_value='0.3',
        description='Speed multiplier for precision mode'
    )
    
    device_path_arg = DeclareLaunchArgument(
        'device_path',
        default_value='/dev/input/js0',
        description='Input device path for controller'
    )
    
    # Controller teleop node
    ps_controller_teleop_node = Node(
        package='orbibot_teleop',
        executable='ps_controller_teleop',
        name='ps_controller_teleop',
        output='screen',
        parameters=[{
            'max_linear_speed': LaunchConfiguration('max_linear_speed'),
            'max_angular_speed': LaunchConfiguration('max_angular_speed'),
            'deadzone': LaunchConfiguration('deadzone'),
            'turbo_multiplier': LaunchConfiguration('turbo_multiplier'),
            'precision_multiplier': LaunchConfiguration('precision_multiplier'),
            'device_path': LaunchConfiguration('device_path'),
        }],
        remappings=[
            ('/cmd_vel', '/cmd_vel'),
        ]
    )
    
    return LaunchDescription([
        max_linear_speed_arg,
        max_angular_speed_arg,
        deadzone_arg,
        turbo_multiplier_arg,
        precision_multiplier_arg,
        device_path_arg,
        ps_controller_teleop_node,
    ])