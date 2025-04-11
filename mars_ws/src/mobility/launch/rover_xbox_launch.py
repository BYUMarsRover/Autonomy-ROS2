from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

''' For driving the rover with the XBOX Controller plugged directly into the rover'''

namespace = 'mobility'
def generate_launch_description():
    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("joysticks"), "/launch/xbox_drive_launch.py"
            ])
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("mobility"), "/launch/rover_drive_launch.py"
            ])
        )
    ])

