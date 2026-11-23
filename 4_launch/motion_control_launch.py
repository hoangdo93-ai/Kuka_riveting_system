#!/usr/bin/env python3
"""
Launch file for KUKA Riveting Motion Control Node

Usage:
    ros2 launch kuka_riveting_system motion_control.launch.py

Note: The Path Planning Node must be running separately for the
/path_planning/plan service to be available (see path_planning.launch.py
or full_system.launch.py to start both together).
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('kuka_riveting_system')
    params_file = os.path.join(pkg_dir, 'config', 'motion_control_params.yaml')

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='motion_control',
        description='Namespace for the motion control node'
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level: debug, info, warn, error'
    )

    motion_control_node = Node(
        package='kuka_riveting_system',
        executable='motion_control_node',
        name='motion_control_node',
        namespace=LaunchConfiguration('namespace'),
        parameters=[params_file],
        arguments=[
            '--ros-args',
            '--log-level', LaunchConfiguration('log_level')
        ],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        namespace_arg,
        log_level_arg,
        motion_control_node,
    ])
