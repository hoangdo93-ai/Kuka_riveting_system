from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='kuka_riveting_system',
            executable='path_planning_node',
            name='path_planning_node',
            parameters=[
                {'flap_width_mm': 24000.0},
                {'flap_height_mm': 3500.0},
            ]
        ),
        Node(
            package='kuka_riveting_system',
            executable='motion_control_node',
            name='motion_control_node',
        ),
        Node(
            package='kuka_riveting_system',
            executable='riveting_supervisor_node',
            name='riveting_supervisor_node',
        ),
    ])