from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

    # Declare the launch arguments
    rover_serial_port_arg = DeclareLaunchArgument('base_serial_port', default_value='/dev/rover/rtk')
    base_host_arg = DeclareLaunchArgument('base_host', default_value='192.168.1.65')
    base_port_arg = DeclareLaunchArgument('base_port', default_value='16140')
    rover_host_arg = DeclareLaunchArgument('rover_host', default_value='192.168.1.20')
    rover_port_arg = DeclareLaunchArgument('rover_port', default_value='16145')

    # Define nodes

    # UBLOX F9P Node for the rover
    f9p = Node(
        package='ublox_read_2',
        executable='ublox_ros',
        namespace='base',
        name='f9p',
        parameters=[
            {'serial_port': LaunchConfiguration('base_serial_port')},
            {'rover_quantity': 1},
            {'local_host': LaunchConfiguration('base_host')},
            {'local_port': LaunchConfiguration('base_port')},
            {'rover_host': LaunchConfiguration('rover_host')},
            {'rover_port': LaunchConfiguration('rover_port')},
            {'params_file': os.path.join(get_package_share_directory('ublox_read_2'), 'params', 'ublox.yaml')}
        ],
        output='screen'
    )

    ld.add_action(rover_serial_port_arg)
    ld.add_action(base_host_arg)
    ld.add_action(base_port_arg)
    ld.add_action(rover_host_arg)
    ld.add_action(rover_port_arg)
    ld.add_action(f9p)
    
    return ld