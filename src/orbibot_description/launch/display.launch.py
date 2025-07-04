#!/usr/bin/env python3
# display.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file to display OrbiBot in RViz with robot state publisher
    """
    
    # Get the package directory
    pkg_path = os.path.join(get_package_share_directory('orbibot_description'))
    
    # Path to the main URDF xacro file
    xacro_file = os.path.join(pkg_path, 'urdf', 'orbibot.urdf.xacro')
    
    # Path to RViz config
    rviz_config_file = os.path.join(pkg_path, 'rviz', 'orbibot.rviz')

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

    # Robot State Publisher node
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

    # Joint State Publisher node - publishes wheel joint states
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # RViz node run from my computer and other node run on robot raspberry pi
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    #     arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
    #     parameters=[{'use_sim_time': use_sim_time}]
    # )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add the declarations
    ld.add_action(declare_use_sim_time)

    # Add the nodes
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    # ld.add_action(rviz_node)

    return ld


if __name__ == '__main__':
    generate_launch_description()