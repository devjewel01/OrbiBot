#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # Launch arguments
    teleop_type = LaunchConfiguration('teleop_type')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_teleop_type = DeclareLaunchArgument(
        'teleop_type',
        default_value='keyboard',
        description='Type of teleop to use: keyboard or simple'
    )
    
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
        emulate_tty=True,
        condition='launch.conditions.IfCondition(launch.substitutions.EqualsSubstitution(teleop_type, "keyboard"))'
    )
    
    # Simple teleop node  
    simple_teleop_node = Node(
        package='orbibot_teleop',
        executable='simple_teleop',
        name='simple_teleop',
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        output='screen',
        emulate_tty=True,
        condition='launch.conditions.IfCondition(launch.substitutions.EqualsSubstitution(teleop_type, "simple"))'
    )
    
    return LaunchDescription([
        declare_teleop_type,
        declare_use_sim_time,
        keyboard_teleop_node,
        simple_teleop_node
    ])