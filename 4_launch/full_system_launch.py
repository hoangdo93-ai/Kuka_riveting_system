#!/usr/bin/env python3
"""
Combined launch file - starts both Path Planning Node (Phase 3) and
Motion Control Node (Phase 4) together.

Usage:
    ros2 launch kuka_riveting_system full_system.launch.py

This is the recommended way to run the system end-to-end: the Motion
Control Node's Action Server depends on the Path Planning Node's
/path_planning/plan service being available.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('kuka_riveting_system')
    path_planning_params = os.path.join(
        pkg_dir, 'config', 'path_planning_params.yaml')
    motion_control_params = os.path.join(
        pkg_dir, 'config', 'motion_control_params.yaml')

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level applied to both nodes: debug, info, warn, error'
    )

    path_planning_node = Node(
        package='kuka_riveting_system',
        executable='path_planning_node',
        name='path_planning_node',
        namespace='path_planning',
        parameters=[path_planning_params],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        output='screen',
        emulate_tty=True,
    )

    motion_control_node = Node(
        package='kuka_riveting_system',
        executable='motion_control_node',
        name='motion_control_node',
        namespace='motion_control',
        parameters=[motion_control_params],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        log_level_arg,
        path_planning_node,
        motion_control_node,
    ])
