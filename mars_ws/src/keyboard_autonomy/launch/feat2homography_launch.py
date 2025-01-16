from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='keyboard_autonomy',
            executable='feat2homography',
            name='feat2homography'
        )
    ])
