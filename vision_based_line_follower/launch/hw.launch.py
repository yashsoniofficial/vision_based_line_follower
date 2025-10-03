from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='line_follower_nodes',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[{'camera_index': 0}]
        ),
        Node(
            package='line_follower_nodes',
            executable='line_detector',
            name='line_detector',
            output='screen'
        ),
        Node(
            package='line_follower_nodes',
            executable='controller_node',
            name='controller_node',
            output='screen',
            parameters=[{
                'use_sim': False,
                'serial_port': '/dev/ttyUSB0',
                'baudrate': 115200,
                'kp': 0.01,
                'kd': 0.002,
                'base_speed': 80
            }]
        ),
    ])
