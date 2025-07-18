#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='keyboard_teleop',
            output='screen',
            remappings=[
                ('/cmd_vel', '/cmd_vel'),
            ],
            parameters=[
                {'stamped': False},
                {'frame_id': ''}
            ]
        )
    ])