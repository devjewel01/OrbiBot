#!/usr/bin/env python3

"""This is for my Computer remotely run to see robot rviz"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    description_dir = get_package_share_directory('orbibot_description')
    
    return LaunchDescription([
        # RViz for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(description_dir, 'rviz', 'orbibot.rviz')]
        ),
        
        # # Optional: rqt for debugging
        # Node(
        #     package='rqt_robot_steering',
        #     executable='rqt_robot_steering',
        #     name='rqt_robot_steering'
        # ),
    ])