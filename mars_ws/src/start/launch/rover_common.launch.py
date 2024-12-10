from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory
import os

# Rover Common Launch
# Created by Daniel Webb Oct 2024

# Launches everything for the rover that is shared among all four tasks.
# This launch file must be called by every base_task_<task_name>.launch file.

def generate_launch_description():

    # Get directories TODO: Uncomment packages as they are created
    odometry_dir = get_package_share_directory('odometry')
    peripherals_dir = get_package_share_directory('peripherals')
    home_gui_dir = get_package_share_directory('home_gui')
    heartbeat_dir = get_package_share_directory('heartbeat')

    return LaunchDescription([
        # Environment variable for ROS console output format
        DeclareLaunchArgument(
            'ROSCONSOLE_FORMAT',
            default_value='(${node})[${severity}]: ${message}',
            description='Console output format'
        ),

        # Node for drive serial communication
        # TODO: needs to be different because of the bridge
        # Node(
        #     package='rosserial_python',
        #     executable='serial_node.py',
        #     name='drive_serial',
        #     parameters=[{
        #         'port': '/dev/rover/onBoardMega',
        #         'baud': 115200
        #     }]
        # ),

        # Include other launch files TODO: Uncomment packages as they are created
        # IncludeLaunchDescription( #TODO: needs to be different
        #     PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/rover_drive.launch.py'])
        # ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join( 
                peripherals_dir, 'launch', 'battery_info.launch.py'))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join( 
                home_gui_dir, 'launch', 'rover_home_gui.launch.py'))
        ),

        # Node for rover status listener
        Node(
            package='peripherals',
            executable='wrapper.py',
            name='rover_status_listener',
            output='screen'
        ),

        # Dummy publisher for rover state data when running locally
        # TODO: add code

        # Arguments for GPS
        DeclareLaunchArgument(
            'rover_host',
            default_value='192.168.1.120',
            # default_value=EnvironmentVariable('ROVER_ADDRESS'),
            description='Rover host address'
        ),
        DeclareLaunchArgument(
            'base_host',
            default_value='',
            # default_value=EnvironmentVariable('BASE_ADDRESS'),
            description='Base host address'
        ),
        DeclareLaunchArgument(
            'rover_serial_port',
            default_value='/dev/rover/rtk',
            description='Rover serial port'
        ),

        # Include GPS related launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                odometry_dir, 'launch', 'rover.launch.py')),
            launch_arguments={
                'rover_host': LaunchConfiguration('rover_host'),
                'base_host': LaunchConfiguration('base_host'),
                'rover_serial_port': LaunchConfiguration('rover_serial_port')
            }.items()
        ),

        # Heartbeat launch file inclusion
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                heartbeat_dir, 'launch', 'heartbeat_rover_launch.py'))
        ),
    ])