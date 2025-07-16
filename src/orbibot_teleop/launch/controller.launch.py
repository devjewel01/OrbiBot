#!/usr/bin/env python3
"""
PlayStation Controller Teleop Launch File for OrbiBot using ROS2 Joy
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for PlayStation controller teleop using joy"""
    
    # Launch arguments
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='/dev/input/js0',
        description='Joystick device path'
    )
    
    deadzone_arg = DeclareLaunchArgument(
        'deadzone',
        default_value='0.05',
        description='Analog stick deadzone'
    )
    
    # Joy node (reads joystick input)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'device': LaunchConfiguration('device'),
            'device_id': 0,
            'deadzone': LaunchConfiguration('deadzone'),
            'autorepeat_rate': 20.0,
        }],
        output='screen'
    )
    
    # PS Controller node (left stick = direction, right stick = speed)
    ps_controller_node = Node(
        package='orbibot_teleop',
        executable='ps_controller_node',
        name='ps_controller_node',
        parameters=[{
            'max_linear_speed': 0.8,     # Max linear speed
            'max_angular_speed': 1.2,    # Max angular speed
            'deadzone': 0.05,            # Stick deadzone
        }],
        remappings=[
            ('cmd_vel', '/cmd_vel'),
        ],
        output='screen'
    )
    
    # Emergency stop node
    emergency_stop_node = Node(
        package='orbibot_teleop',
        executable='emergency_stop_node',
        name='emergency_stop_node',
        output='screen',
        remappings=[
            ('/joy', '/joy'),  # Subscribe to joy messages
        ]
    )
    
    return LaunchDescription([
        device_arg,
        deadzone_arg,
        joy_node,
        ps_controller_node,
        emergency_stop_node,  # Emergency stop functionality
    ])