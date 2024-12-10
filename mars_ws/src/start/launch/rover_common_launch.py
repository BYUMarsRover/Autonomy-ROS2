from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

# Rover Common Launch

# Launches everything for the rover that is shared among all four tasks.
# This launch file must be called by every base_task_<task_name>.launch file.

def generate_launch_description():

    # Get directories TODO: Uncomment packages as they are created
    # peripherals_dir = get_package_share_directory('peripherals')
    odometry_dir = get_package_share_directory('odometry')
    mobilility_dir = get_package_share_directory('mobility')
    home_gui_dir = get_package_share_directory('home_gui')
    heartbeat_dir = get_package_share_directory('heartbeat')

    return LaunchDescription([
        # Environment variable for ROS console output format
        DeclareLaunchArgument(
            'ROSCONSOLE_FORMAT',
            default_value='(${node})[${severity}]: ${message}',
            description='Console output format'
        ),

        # Dummy publisher for rover state data when running locally
        # TODO: add code

        # Node for drive serial communication
        # TODO: needs to be different because of the bridge
        # Serial communcication with the Mega Arduino
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(os.path.join( 
        #         peripherals_dir, 'launch', 'battery_info.launch.py'))
        # ),

        # Node for rover status listener TODO: Uncomment once created
        # Node(
        #     package='peripherals',
        #     executable='wrapper',
        #     name='rover_status_listener',
        #     output='screen'
        # ),

        # Heartbeat lrover node
        Node(
            package='heartbeat',
            executable='heartbeat_rover',
            name='heartbeat_rover',
            output='screen',
        ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(os.path.join( 
        #         home_gui_dir, 'launch', 'rover_home_gui.launch.py'))
        # ),

        # Rover Translator node for mobility messages
        Node(
            package='mobility',
            executable='transition',
            name='transition',
            output='screen',
            namespace='mobility',
        ),

        # Include GPS related launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                odometry_dir, 'launch', 'rover_launch.py')),
        ),

    ])