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
    mapviz_location=os.environ.get('MAPVIZ_LOCATION', '')
    mapviz_location_arg = DeclareLaunchArgument('MAPVIZ_LOCATION', default_value=mapviz_location)

    rover_address=os.environ.get('ROVER_ADDRESS', '')
    rover_address_arg = DeclareLaunchArgument('ROVER_ADDRESS', default_value=rover_address)

    # Start all common launch files on the rover
    rover_common_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PythonLaunchDescriptionSource([
                FindPackageShare("start"), "/launch/rover_common_launch.py"
            ])
        ),
        launch_arguments={
            'ROVER_ADDRESS': LaunchConfiguration('ROVER_ADDRESS')
        }.items()
    )

    # Start launch files specific to the Autonomy Task on the rover
    autonomy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
                FindPackageShare("autonomy"), "/launch/base_launch.py"
            ]),
        launch_arguments={
            'location': LaunchConfiguration('MAPVIZ_LOCATION')
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
        PythonLaunchDescriptionSource(
            PythonLaunchDescriptionSource([
                FindPackageShare("odometry"), "/launch/estimation_launch.py"
            ])
        ),
        launch_arguments={
            'ROVER_ADDRESS': LaunchConfiguration('ROVER_ADDRESS')
        }.items()
    )

    return LaunchDescription([
        mapviz_location_arg,
        rover_address_arg,
        rover_common_launch,
        autonomy_launch,
        autopilot_drive_launch,
        estimation_launch
    ])