from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='keyboard_autonomy',
            executable='homography2keys.py',
            name='homography2keys'
        )
    ])
