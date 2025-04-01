from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="science",
            executable="science_serial_interface",
            name="science_serial_interface",
            output="screen"
        ),
        # Node(
        #     package="science",
        #     executable="science_control",
        #     name="science_control",
        #     output="screen"
        # ),
        Node(
            package="science",
            executable="science_GUI",
            name="science_GUI",
            output="screen"
        ),
        Node(
            package="science",
            executable="science_rxtx",
            name="science_rxtx",
            output="screen"
        ),
        Node(
            package="science",
            executable="science_response",
            name="science_response",
            output="screen"
        ),
        Node(
            package="science",
            executable="science_debug",
            name="science_debug",
            output="screen"
        ),
        Node(
            package="science",
            executable="science_data_saver",
            name="science_data_saver",
            output="screen"
        )
    ])
