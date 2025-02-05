# ===================================
# ==== Rover Science Task Launch=====
# ===================================

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # import environment variables
    mapviz_location=os.environ.get('MAPVIZ_LOCATION', '')
    mapviz_location_arg = DeclareLaunchArgument('MAPVIZ_LOCATION', default_value=mapviz_location)

    return LaunchDescription([
        # Start all common launch files on the rover. DO NOT TOUCH
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("start"), "/launch/rover_common_launch.py"
            ])
        ),

        # Start launch files specific to the Science Task on the rover
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("science"), "/launch/rover_launch.py"
            ])
        ),

        # Launch UKF (Unscented Kalman Filter) so that GPS data will work on the science GUI TODO - may need to update to reflect autonomy changes.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("odometry"), "/launch/estimation_launch.py"
            ])
        ),
        mapviz_location_arg,
    ])