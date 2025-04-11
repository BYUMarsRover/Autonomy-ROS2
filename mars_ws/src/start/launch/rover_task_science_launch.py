# ===================================
# ==== Rover Science Task Launch=====
# ===================================

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Start all common launch files on the rover.
    common_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("start"), "/launch/rover_common_launch.py"
        ])
    )

    # Launch the Science Serial Interface and other rover side science nodes
    science_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("science"), "/launch/rover_launch.py"
        ])
    )

    # Launch UKF (Unscented Kalman Filter) so that GPS data will work on the science GUI TODO - may need to update to reflect autonomy changes.
    odometry_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("odometry"), "/launch/estimation_launch.py"
        ])
    )

    return LaunchDescription([
        common_launch,
        science_launch
        #odometry_launch - Determine if odometry is needed for the science task
    ])