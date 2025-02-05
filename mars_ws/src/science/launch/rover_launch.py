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
        Node(
            package="science",
            executable="science_control",
            name="science_control",
            output="screen"
        )
        # Node(
        #     package="science",
        #     executable="science_GUI",
        #     name="science_GUI",
        #     output="screen"
        # ),
        # Node(
        #     package="science",
        #     executable="science_data_saver",
        #     name="science_data_saver",
        #     output="screen"
        # )
    ])
