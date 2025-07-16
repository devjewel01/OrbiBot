#!/usr/bin/env python3

import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package directories
    pkg_orbibot_simulation = get_package_share_directory('orbibot_simulation')
    pkg_orbibot_description = get_package_share_directory('orbibot_description')
    
    # Set Gazebo model path to resolve package:// URIs
    gazebo_models_path, _ = os.path.split(pkg_orbibot_description)
    os.environ["GZ_SIM_RESOURCE_PATH"] = os.environ.get("GZ_SIM_RESOURCE_PATH", "") + os.pathsep + gazebo_models_path
    
    # Launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty',
        description='Gazebo world to load (empty, office)'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start Gazebo GUI'
    )
    
    x_arg = DeclareLaunchArgument(
        'x', default_value='0.0', description='Initial X position'
    )
    
    y_arg = DeclareLaunchArgument(
        'y', default_value='0.0', description='Initial Y position'
    )
    
    z_arg = DeclareLaunchArgument(
        'z', default_value='0.1', description='Initial Z position'
    )
    
    yaw_arg = DeclareLaunchArgument(
        'yaw', default_value='0.0', description='Initial Yaw'
    )
    
    # World file path
    world_file_path = PathJoinSubstitution([
        FindPackageShare('orbibot_simulation'),
        'worlds',
        [LaunchConfiguration('world'), '.sdf']
    ])
    
    # Process robot description using xacro
    robot_model_path = os.path.join(pkg_orbibot_description, 'urdf', 'orbibot.urdf.xacro')
    robot_description = xacro.process_file(robot_model_path).toxml()
    
    # Gazebo bridge config
    gz_bridge_params_path = os.path.join(pkg_orbibot_simulation, 'config', 'gz_bridge.yaml')
    
    # Start Gazebo using ros_gz_sim
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': ['-r -v 4 ', world_file_path],
            'on_exit_shutdown': 'true'
        }.items()
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_orbibot',
        arguments=[
            '-name', 'orbibot',
            '-string', robot_description,
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-Y', LaunchConfiguration('yaw'),
            '-allow_renaming', 'false'
        ],
        output='screen'
    )
    
    # ROS-Gazebo bridge
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        arguments=[
            '--ros-args', '-p',
            f'config_file:={gz_bridge_params_path}'
        ],
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    return LaunchDescription([
        # Arguments
        world_arg,
        use_sim_time_arg,
        gui_arg,
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        
        # Nodes
        gazebo_launch,
        robot_state_publisher,
        spawn_robot,
        gz_bridge,
    ])