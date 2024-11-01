from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='peripherals',
            executable='battery_info',
            name='battery_info',
            output='screen'
        )
    ])