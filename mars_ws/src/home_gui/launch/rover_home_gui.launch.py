from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='home_gui',
            executable='rover_dev_update',
            name='rover_dev_update',
            output='screen'
        ),
        Node(
            package='home_gui',
            executable='rover_camera_control',
            name='rover_camera_control',
            output='screen'
        )
    ])