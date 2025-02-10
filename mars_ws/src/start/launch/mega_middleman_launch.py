
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

namespace = 'mobility'

def generate_launch_description():

    mobility_dir = get_package_share_directory('mobility')

    return LaunchDescription([
        # Environment variable for ROS console output format
        DeclareLaunchArgument(
            'ROSCONSOLE_FORMAT',
            default_value='(${node})[${severity}]: ${message}',
            description='Console output format'
        ),

        Node(
                package='mobility',
                executable='mega_middleman',
                name='mega_middleman',
                output='screen',
                namespace=namespace,
                )
    ])