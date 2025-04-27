from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    ublox_config = os.path.join(get_package_share_directory('ublox_read_2'), 'params', 'ublox.yaml')

    return LaunchDescription([
        # UBLOX F9P Node for the base
        Node(
            package='ublox_read_2',
            executable='ublox_ros',
            namespace='base',
            name='f9p_base',
            parameters=[
                ublox_config
            ],
            output='screen'
        ),
    ])