
# ===================================
# ==== Rover Autonomy Task Launch====
# ===================================

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # import environment variables
    mapviz_location = os.environ.get('MAPVIZ_LOCATION', 'hanksville')
    mapviz_location_arg = DeclareLaunchArgument('MAPVIZ_LOCATION', default_value=mapviz_location)

    # Start all common launch files on the rover
    rover_common_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
                FindPackageShare("start"), "/launch/rover_common_launch.py"
            ])
    )

    # Start launch files specific to the Autonomy Task on the rover
    autonomy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
                FindPackageShare("autonomy"), "/launch/rover_launch.py"
            ]),
        launch_arguments={
            'MAPVIZ_LOCATION': LaunchConfiguration('MAPVIZ_LOCATION')
        }.items()
    )

    # TODO: Add when converted to ROS2
    # Start Mobility low level nodes
    autopilot_drive_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
                FindPackageShare("mobility"), "/launch/autopilot_drive_launch.py"
            ])
    )

    # Start localization in odometry
    estimation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
                FindPackageShare("odometry"), "/launch/estimation_new_launch.py"
            ])
    )

    return LaunchDescription([
        mapviz_location_arg,
        rover_common_launch,
        autonomy_launch,
        autopilot_drive_launch,
        estimation_launch
    ])