from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='traffic_light_detector',
            executable='traffic_light_detector',
            name='traffic_light_detector',
            output='screen',
        ),
        Node(
            package='basic_agent',
            executable='basic_agent_node',
            name='basic_agent',
            output='screen',
        ),
    ])
