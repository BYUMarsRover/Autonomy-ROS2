
# ===================================
# ==== Rover Autonomy Task Launch====
# ===================================

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Start all common launch files on the rover
    include_rover_common = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('start'),
                'launch',
                'rover_common_launch.py'
            )
        ])
    )

    # Start launch files specific to the Autonomy Task on the rover
    include_autonomy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('autonomy'),
                'launch',
                'autonomy_launch.py'
            )
        ])
    )

    # TODO: Add when converted to ROS2
    # Start Mobility low level nodes
    include_autopilot_drive = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('mobility'),
                'launch',
                'autopilot_drive.launch.py'
            )
        ])
    )

    # Start localization in odometry
    include_estimation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('odometry'),
                'launch',
                'estimation_launch.py'
            )
        ])
    )

    return LaunchDescription([
        include_rover_common,
        include_autonomy,
        include_autopilot_drive,
        include_estimation
    ])