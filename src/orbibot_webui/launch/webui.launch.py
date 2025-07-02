#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Launch arguments
    host = LaunchConfiguration('host')
    port = LaunchConfiguration('port')
    debug = LaunchConfiguration('debug')
    
    declare_host = DeclareLaunchArgument(
        'host',
        default_value='0.0.0.0',
        description='Web server host address (default: 0.0.0.0 for all interfaces)'
    )
    
    declare_port = DeclareLaunchArgument(
        'port',
        default_value='5000',
        description='Web server port (default: 5000)'
    )
    
    declare_debug = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable Flask debug mode'
    )
    
    # Web monitor node
    web_monitor_node = Node(
        package='orbibot_webui',
        executable='web_monitor',
        name='orbibot_web_monitor',
        arguments=[
            '--host', host,
            '--port', port
        ],
        output='screen',
        emulate_tty=True
    )
    
    return LaunchDescription([
        declare_host,
        declare_port,
        declare_debug,
        web_monitor_node
    ])