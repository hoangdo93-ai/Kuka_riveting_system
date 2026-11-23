#!/usr/bin/env python3
"""
Launch file for KUKA Riveting Path Planning Node

Usage:
    ros2 launch kuka_riveting_system path_planning.launch.py

This launch file:
1. Loads default parameters from config/path_planning_params.yaml
2. Starts the path_planning_node executable
3. Enables ROS2 logging
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Generate launch description for path planning node
    
    Returns:
        LaunchDescription with path_planning_node configuration
    """
    
    # Get package directory
    pkg_dir = get_package_share_directory('kuka_riveting_system')
    
    # Path to parameter file
    params_file = os.path.join(pkg_dir, 'config', 'path_planning_params.yaml')
    
    # Declare launch arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='path_planning',
        description='Namespace for the path planning node'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level: debug, info, warn, error'
    )
    
    # Path Planning Node
    path_planning_node = Node(
        package='kuka_riveting_system',
        executable='path_planning_node',
        name='path_planning_node',
        namespace=LaunchConfiguration('namespace'),
        parameters=[params_file],
        arguments=[
            '--ros-args',
            '--log-level', LaunchConfiguration('log_level')
        ],
        output='screen',
        emulate_tty=True,
    )
    
    # Return launch description with all components
    return LaunchDescription([
        namespace_arg,
        log_level_arg,
        path_planning_node,
    ])
