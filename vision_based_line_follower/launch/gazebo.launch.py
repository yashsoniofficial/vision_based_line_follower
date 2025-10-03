from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os

def generate_launch_description():
    world_path = os.path.join(
        os.path.dirname(__file__), '../../../gazebo/track_world.sdf'
    )

    return LaunchDescription([
        # Start Gazebo with world
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_path, '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # ROS nodes
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
            parameters=[{'use_sim': True}]
        )
    ])
