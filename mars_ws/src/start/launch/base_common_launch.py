from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

# Base Common Launch

# Launches everything for the base that is shared among all four tasks. 
# This launch file must be called by every base_task_<task_name>_launch file.


def generate_launch_description():
    # Define launch arguments

    # Set environment variable TODO: Is this needed?
    # set_rosconsole_format = SetEnvironmentVariable(
    #     'ROSCONSOLE_FORMAT', '(${node})[${severity}]: ${message}'
    # )

    # Include other launch files TODO: Uncomment packages as they are created
    include_xbox_drive = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('mobility'), 'launch', 'xbox_drive_launch.py'))
    )

    include_base_home_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('home_gui'), 'launch', 'base_home_gui.launch.py'))
    )

    include_heartbeat_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('heartbeat'), 'launch', 'heartbeat_base_launch.py'))
    )

    include_mapviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('mapviz_tf'), 'launch', 'mapviz_launch.py'))
    )

    #GPS node on the base station
    include_odometry_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('odometry'), 'launch', 'base_launch.py')),
    )

    return LaunchDescription([
        # set_rosconsole_format,
        include_xbox_drive,
        # include_base_home_gui,
        include_heartbeat_base,
        # include_mapviz,
        include_odometry_base
    ])