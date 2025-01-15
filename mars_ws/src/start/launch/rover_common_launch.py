from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
#(imports below needed for running the Dummy Publisher at the end of the file)
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


# Rover Common Launch

# Launches everything for the rover that is shared among all four tasks.
# This launch file must be called by every base_task_<task_name>.launch file.

def generate_launch_description():

    # Get directories TODO: Uncomment packages as they are created
    # peripherals_dir = get_package_share_directory('peripherals')
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

        # Peripherals 
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join( 
                peripherals_dir, 'launch', 'peripherals_launch.py'))
        ),

        # Heartbeat
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join( 
                heartbeat_dir, 'launch', 'heartbeat_rover_launch.py'))
        ),

        # Rover Home GUI
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join( 
                home_gui_dir, 'launch', 'rover_home_gui.launch.py'))
        ),

         # Node for drive serial communication
        # serial communication = IWC_motors -> subscribes to the end of the mobility module (transisiton node?) and
        # we are going to use a ros2 bridge to go back and forth between the ros2 mobility and motors
        # TODO: needs to be different because of the bridge
        # Serial communcication with the Mega Arduino (tells what the motors need to do)
        # Launch Mobility Package - Rover Translator node for mobility messages
        # We aren't sure if this will work in ROS2, we might need to make the Arduino & Computer communication ros indepenent 
        # before we can get mobility working in ROS2. Or maybe we can add something to make the bridge. 
        # Mobility
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                mobility_dir, 'launch', 'rover_drive_launch.py')),
        ),

        # GPS
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                odometry_dir, 'launch', 'rover_launch.py')),
        ),

        # Dummy publisher for rover state data when running locally---------------
        # This is only needed if the os.getenv is http://127.0.0.1:11311
        GroupAction(
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(odometry_dir, 'launch', 'dummy_singleton_publisher.launch.py')
                    )
                )
            ],
            condition=IfCondition( #if that condition is met
                LaunchConfiguration(
                    'ros_master_uri', default=os.getenv('ROS_MASTER_URI', '')
                ) == 'http://127.0.0.1:11311'
            )
        ),
        #-----------------------------------------

    ])