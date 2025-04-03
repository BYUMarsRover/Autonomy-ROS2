from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    ublox_config = os.path.join(get_package_share_directory('ublox_read_2'), 'params', 'ublox.yaml')

    return LaunchDescription([
        #What is this?
        # DeclareLaunchArgument('standalone', default_value='false'),
        
        # Node(
        #     package='odometry',
        #     executable='position_velocity_time_translator',
        #     namespace='rover',
        #     name='position_velocity_time_translator',
        #     remappings=[
        #         ('lla', '/ins/lla')
        #     ],
        #     output='screen'
        # ),
        Node(
            package='ublox_read_2',
            executable='ublox_ros',
            namespace='rover',
            name='f9p_rover',
            parameters=[ublox_config],
            output='screen'
        ),
        #TODO: ADD STATIC TRANSFORM PUBLISHER FOR GPS TO BASE LINK FRAME
    ])