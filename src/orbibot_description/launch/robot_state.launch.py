#!/usr/bin/env python3
# robot_state.launch.py - Launch file for real robot state publishing
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file for robot state publisher only (for real robot)
    Use this when running the real robot - it expects joint_states from hardware
    """
    
    # Get the package directory
    pkg_path = os.path.join(get_package_share_directory('orbibot_description'))
    
    # Path to the main URDF xacro file
    xacro_file = os.path.join(pkg_path, 'urdf', 'orbibot.urdf.xacro')

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use sim time if true'
    )
    

    # Process the URDF file
    robot_description = Command(['xacro ', xacro_file])

    # Robot State Publisher node - publishes tf based on joint_states
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add the declarations
    ld.add_action(declare_use_sim_time)

    # Add the nodes
    ld.add_action(robot_state_publisher_node)

    return ld


if __name__ == '__main__':
    generate_launch_description()