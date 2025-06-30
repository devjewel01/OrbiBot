#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    # Keyboard teleop node
    keyboard_teleop_node = Node(
        package='orbibot_teleop',
        executable='keyboard_teleop',
        name='keyboard_teleop',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'max_linear_speed': 1.0},
            {'max_angular_speed': 1.5},
            {'speed_step': 0.1},
            {'angular_step': 0.2}
        ],
        output='screen',
        emulate_tty=True
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        keyboard_teleop_node
    ])