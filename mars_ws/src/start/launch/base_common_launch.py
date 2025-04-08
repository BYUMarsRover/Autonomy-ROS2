# ===================================
# ====== Base Commmon Launch ========
# ===================================

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
import os

# Base Common Launch

# Launches everything for the base that is shared among all four tasks. 
# This launch file must be called by every base_task_<task_name>_launch file.


def generate_launch_description():
    return

    # Declare launch arguments
    mapviz_location_arg = DeclareLaunchArgument('MAPVIZ_LOCATION', default_value='hanksville')

    # Set Log Info for Debugging
    set_rosconsole_format = SetEnvironmentVariable(
        'ROSCONSOLE_FORMAT', '(${node})[${severity}]: ${message}'
    )

    # Launch the XBOX controller for drive
    xbox_drive_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("joysticks"), "/launch/xbox_drive_launch.py"
        ])
    )

    # Launch the Home GUI
    base_home_gui_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("home_gui"), "/launch/base_home_gui.launch.py"
        ])
    )

    # Launch the heartbeat
    heartbeat_base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("heartbeat"), "/launch/heartbeat_base_launch.py"
        ])
    )

    # Launch MAPVIZ
    mapviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("mapviz_tf"), "/launch/mapviz_launch.py"
        ]),
        launch_arguments={
            'MAPVIZ_LOCATION': LaunchConfiguration('MAPVIZ_LOCATION')
        }.items()
    )

    # GPS node on the base station
    odometry_base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("odometry"), "/launch/base_launch.py"
        ])
    )

    return LaunchDescription([
        mapviz_location_arg,
        set_rosconsole_format,
        xbox_drive_launch,
        base_home_gui_launch,
        heartbeat_base_launch,
        mapviz_launch,
        odometry_base_launch
    ])