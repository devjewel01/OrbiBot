#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    mode_arg = DeclareLaunchArgument('mode', default_value='slam')
    mode = LaunchConfiguration('mode')
    
    return LaunchDescription([mode_arg])