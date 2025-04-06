import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package="science",
            executable="science_GUI",
            name="science_GUI",
            output="screen"
        ),
        Node(
            package="science",
            executable="science_data_saver",
            name="science_data_saver",
            output="screen"
        )
    ])
