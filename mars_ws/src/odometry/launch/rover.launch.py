from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

    # Declare the launch arguments
    standalone_arg = DeclareLaunchArgument('standalone', default_value='false')
    rover_serial_port_arg = DeclareLaunchArgument('rover_serial_port', default_value='/dev/ttyACM0')
    base_host_arg = DeclareLaunchArgument('base_host', default_value='') #TODO: set a default value
    base_port_arg = DeclareLaunchArgument('base_port', default_value='16140')
    rover_host_arg = DeclareLaunchArgument('rover_host', default_value='192.168.1.120') #TODO: set a default value
    rover_port_arg = DeclareLaunchArgument('rover_port', default_value='16145')

    # Define nodes

    # UBLOX F9P Node for the rover
    f9p = Node(
        package='ublox_read_2',
        executable='ublox_ros',
        namespace='rover',
        name='f9p',
        parameters=[
            {'serial_port': LaunchConfiguration('rover_serial_port')},
            {'rover_quantity': 0},
            {'local_host': LaunchConfiguration('rover_host')},
            {'local_port': LaunchConfiguration('rover_port')},
            {'base_host': LaunchConfiguration('base_host')},
            {'base_port': LaunchConfiguration('base_port')},
            {'params_file': os.path.join(get_package_share_directory('ublox_read_2'), 'params', 'ublox.yaml')}
        ],
        output='screen'
    )

    # Position Velocity Time Translator Node
    position_velocity_time_translator = Node(
        package='odometry',
        executable='position_velocity_time_translator',
        namespace='rover',
        name='position_velocity_time_translator',
        remappings=[
            ('lla', '/ins/lla')
        ],
        output='screen'
    )

    ld.add_action(standalone_arg)
    ld.add_action(rover_serial_port_arg)
    ld.add_action(base_host_arg)
    ld.add_action(base_port_arg)
    ld.add_action(rover_host_arg)
    ld.add_action(rover_port_arg)
    ld.add_action(f9p)
    ld.add_action(position_velocity_time_translator)
    
    return ld