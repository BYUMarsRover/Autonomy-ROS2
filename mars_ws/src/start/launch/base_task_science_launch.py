# ==================================
# ==== Base Science Task Launch=====
# ==================================

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    mapviz_location=os.environ.get('MAPVIZ_LOCATION', '')
    mapviz_location_arg = DeclareLaunchArgument('MAPVIZ_LOCATION', default_value=mapviz_location)

    return LaunchDescription([
        # Start all common launch files on the base station. DO NOT TOUCH
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("start"), "/launch/base_common_launch.py"
            ])
        ),

        # Start launch files specific to the Science Task on the base station
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
        mapviz_location_arg, #TODO - consider changing this so that everything matches here or changing the others so it matches the style of autonomy
        # include_base_common,
    ])