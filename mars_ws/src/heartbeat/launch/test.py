from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='heartbeat',
            executable='HeartbeatBase',
            name='heartbeat_base',
            output='screen'
        ),
        Node(
            package='heartbeat',
            executable='HeartbeatRover',
            name='heartbeat_rover',
            output='screen'
        )
    ])
