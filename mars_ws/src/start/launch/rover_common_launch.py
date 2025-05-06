from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

# Rover Common Launch

# Launches everything for the rover that is shared among all four tasks.

def generate_launch_description():

    odometry_dir = get_package_share_directory('odometry')
    mobility_dir = get_package_share_directory('mobility')
    home_gui_dir = get_package_share_directory('home_gui')
    heartbeat_dir = get_package_share_directory('heartbeat')
    peripherals_dir = get_package_share_directory('peripherals')

    return LaunchDescription([
        # Environment variable for ROS console output format
        DeclareLaunchArgument(
            'ROSCONSOLE_FORMAT',
            default_value='(${node})[${severity}]: ${message}',
            description='Console output format'
        ),
        DeclareLaunchArgument('ROVER_ADDRESS', default_value='192.168.1.120'),

        # NOTE: Comment not pushed because it is a temporary fix
        # Peripherals
        # Moved to USB devices launch file
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(os.path.join( 
        #         peripherals_dir, 'launch', 'peripherals.launch.py'))
        # ),

        # Heartbeat
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(os.path.join( 
        #         heartbeat_dir, 'launch', 'heartbeat_rover_launch.py'))
        # ),

        # # Rover Home GUI
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(os.path.join( 
        #         home_gui_dir, 'launch', 'rover_home_gui.launch.py'))
        # ),

        # Mobility
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(os.path.join(
        #         mobility_dir, 'launch', 'rover_drive_launch.py'))
        # ),

        # GPS # LAUNCHED SEPERATELY for MOCK in PRICE
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(os.path.join(
        #         odometry_dir, 'launch', 'rover_launch.py'))
        # ),

        # # Dummy publisher for rover state data when running locally
        GroupAction(
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(odometry_dir, 'launch', 'dummy_singleton_publisher.launch.py')
                    )
                )
            ],
            condition=IfCondition(
                PythonExpression(["'", LaunchConfiguration('ROVER_ADDRESS'), "' == '127.0.0.1'"])
            )
        ),
    ])
