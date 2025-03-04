# ===================================
# ==== Rover Science Task Launch=====
# ===================================

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # import environment variables
    mapviz_location=os.environ.get('MAPVIZ_LOCATION', '')
    mapviz_location_arg = DeclareLaunchArgument('MAPVIZ_LOCATION', default_value=mapviz_location)
    home_gui_dir = get_package_share_directory('home_gui')
    peripherals_dir = get_package_share_directory('peripherals')

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

        # Peripherals
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join( 
                peripherals_dir, 'launch', 'peripherals.launch.py'))
        ),

        # Rover Home GUI
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join( 
                home_gui_dir, 'launch', 'rover_home_gui.launch.py'))
        ),

        # Launch UKF (Unscented Kalman Filter) so that GPS data will work on the science GUI TODO - may need to update to reflect autonomy changes.
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         FindPackageShare("odometry"), "/launch/estimation_launch.py"
        #     ])
        # ),
        mapviz_location_arg,
    ])