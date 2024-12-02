from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='home_gui',
            executable='base_home_gui',  # Ensure this matches the executable name
            name='base_home_gui',
            output='screen'
        )
    ])