from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from ament_index_python.packages import get_package_share_directory
import os
import socket

# Base Common Launch
# Created by Daniel Webb Oct 2024

# Launches everything for the base that is shared among all four tasks. 
# This launch file must be called by every base_task_<task_name>.launch file.

def get_local_ip():
    """Retrieve the local IP address of the machine."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # Connect to an external server to determine local IP address
        s.connect(("8.8.8.8", 80))
        ip_address = s.getsockname()[0]
    finally:
        s.close()
    return ip_address

def generate_launch_description():
    # Define launch arguments
    rover_host_arg = DeclareLaunchArgument(
        'rover_host', default_value='192.168.1.20', #TODO: Change to be set as environment variable in the launch.sh bash script
        description='Rover host address'
    )

    local_ip = get_local_ip()
    base_host_arg = DeclareLaunchArgument(
        'base_host', default_value=local_ip, #TODO: Change to be set as environment variable in the launch.sh bash script
        description='Base host address'
    )
    base_serial_port_arg = DeclareLaunchArgument(
        'base_serial_port', default_value='/dev/rover/rtk',
        description='Base serial port'
    )

    # Set environment variable TODO: Is this needed?
    # set_rosconsole_format = SetEnvironmentVariable(
    #     'ROSCONSOLE_FORMAT', '(${node})[${severity}]: ${message}'
    # )

    # Include other launch files TODO: Uncomment packages as they are created
    # include_xbox_drive = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(
    #         get_package_share_directory('mobility'), 'launch', 'xbox_drive.launch.py'))
    # )

    include_base_home_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('home_gui'), 'launch', 'base_home_gui.launch.py'))
    )

    include_heartbeat_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('heartbeat'), 'launch', 'heartbeat_base_launch.py'))
    )

    # include_mapviz = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(
    #         get_package_share_directory('mapviz_tf'), 'launch', 'mapviz.launch.py'))
    # )

    include_odometry_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('odometry'), 'launch', 'base.launch.py')),
        launch_arguments={
            'rover_host': LaunchConfiguration('rover_host'),
            'base_host': LaunchConfiguration('base_host'),
            'base_serial_port': LaunchConfiguration('base_serial_port')
        }.items()
    )

    return LaunchDescription([
        rover_host_arg,
        base_host_arg,
        base_serial_port_arg,
        # set_rosconsole_format,
        # include_xbox_drive,
        include_base_home_gui,
        include_heartbeat_base,
        # include_mapviz,
        include_odometry_base
    ])