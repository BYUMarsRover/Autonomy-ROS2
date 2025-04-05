# ==================================
# ==== Base Science Task Launch=====
# ==================================

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    mapviz_location=os.environ.get('MAPVIZ_LOCATION', '')
    mapviz_location_arg = DeclareLaunchArgument('MAPVIZ_LOCATION', default_value=mapviz_location)

    return LaunchDescription([
        # Start all common launch files on roverDeviceListthe base station. DO NOT TOUCH
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('home_gui'), 'launch', 'base_home_gui.launch.py'))
            ),   

        # Start launch files specific to the Science Task on the base station
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("science"), "/launch/base_launch.py"
            ])
        ),

        # Start launch files specific to the Science Task on the rocver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("science"), "/launch/rover_launch.py"
            ])
        ),

        mapviz_location_arg, #TODO - consider changing this so that everything matches here or changing the others so it matches the style of autonomy
        # include_base_common,
    ])