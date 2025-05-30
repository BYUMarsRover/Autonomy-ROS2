from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="science",
            executable="science_serial_interface",
            name="science_serial_interface",
            emulate_tty=True, # Allow prints for debug
            output="screen"
        ),
        Node(
            package="science",
            executable="science_request_manager",
            name="science_request_manager",
            emulate_tty=True, # Allow prints for debug
            output="screen"
        )
    ])
