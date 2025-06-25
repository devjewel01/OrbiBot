#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('orbibot_control')
    
    # Configuration file paths
    control_config = os.path.join(pkg_dir, 'config', 'control_params.yaml')
    mecanum_config = os.path.join(pkg_dir, 'config', 'mecanum_params.yaml')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    # Mecanum controller node
    mecanum_controller_node = Node(
        package='orbibot_control',
        executable='mecanum_controller',
        name='mecanum_controller',
        parameters=[
            control_config,
            mecanum_config,
            {'use_sim_time': use_sim_time}
        ],
        output='screen',
        emulate_tty=True
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        mecanum_controller_node
    ])