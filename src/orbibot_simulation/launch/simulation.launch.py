#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    # Get package directories
    pkg_orbibot_simulation = get_package_share_directory('orbibot_simulation')
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty',
        description='Gazebo world to load'
    )
    
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start Gazebo GUI'
    )
    
    use_gazebo_arg = DeclareLaunchArgument(
        'use_gazebo',
        default_value='true',
        description='Use Gazebo simulation'
    )
    
    use_hardware_arg = DeclareLaunchArgument(
        'use_hardware',
        default_value='true',
        description='Use simulation hardware interface'
    )
    
    use_control_arg = DeclareLaunchArgument(
        'use_control',
        default_value='true',
        description='Use simulation control'
    )
    
    # Include Gazebo simulation
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_orbibot_simulation, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'gui': LaunchConfiguration('gui'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_gazebo'))
    )
    
    # Simulation hardware interface
    simulation_hardware_node = Node(
        package='orbibot_simulation',
        executable='simulation_hardware_node',
        name='orbibot_simulation_hardware_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'cmd_timeout': 2.0,
            'publish_rate': 50.0,
            'joint_state_rate': 50.0,
            'system_status_rate': 5.0,
            'safety_status_rate': 10.0,
            'wheel_radius': 0.05,
            'wheel_separation_width': 0.30,
            'wheel_separation_length': 0.18,
            'max_linear_velocity': 1.0,
            'max_angular_velocity': 2.0,
            'battery_voltage_nominal': 12.0,
            'battery_voltage_warning': 11.0,
            'battery_voltage_critical': 10.5,
        }],
        condition=IfCondition(LaunchConfiguration('use_hardware'))
    )
    
    # Simulation control node
    simulation_control_node = Node(
        package='orbibot_simulation',
        executable='simulation_control_node',
        name='orbibot_simulation_control_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'child_frame': 'base_link',
            'publish_rate': 50.0,
            'wheel_radius': 0.05,
            'wheel_separation_width': 0.30,
            'wheel_separation_length': 0.18,
            'enable_tf': True,
            'enable_odom': True,
        }],
        condition=IfCondition(LaunchConfiguration('use_control'))
    )
    
    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        world_arg,
        gui_arg,
        use_gazebo_arg,
        use_hardware_arg,
        use_control_arg,
        
        # Includes and nodes
        gazebo_sim,
        simulation_hardware_node,
        simulation_control_node,
    ])