# ==================================
# ==== Base Science Task Launch=====
# ==================================

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
import os

def generate_launch_description():
    # mapviz_location=os.environ.get('MAPVIZ_LOCATION', '')
    # mapviz_location_arg = DeclareLaunchArgument('MAPVIZ_LOCATION', default_value=mapviz_location)

    return LaunchDescription([
        # Todo work on this with someone who understands it
        # mapviz_location_arg,

        # # Start all common launch files on the rover
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         FindPackageShare("start"), "/launch/base_common_launch.py"
        #     ])
        # ),

        # Launch the science package
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("science"), "/launch/base_launch.py"
            ])
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("joysticks"), "/launch/xbox_science_launch.py"
            ])
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("mobility"), "/launch/xbox_drive_launch.py"
            ])
        )
    ])