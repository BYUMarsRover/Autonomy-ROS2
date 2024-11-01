import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='odometry',
            executable='dummy_singleton_publisher',
            name='dummy_singleton_publisher',
            output='screen',
        ),
    ])