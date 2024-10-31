from launch import LaunchDescription
import launch
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import TextSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define launch arguments
    # skip_arg = DeclareLaunchArgument('skip', default_value='3')
    # display_arg = DeclareLaunchArgument('display', default_value='false')
    # zed_type_arg = DeclareLaunchArgument('zed_type', default_value='zed2')
    config_file = os.path.join(
        get_package_share_directory('navigation'), 'params', 'config.yaml')

    # Path to the ZED launch script
    zed_launch_path = os.path.join(
        get_package_share_directory('zed_wrapper'),
        'launch',
        'zed_camera.launch.py'  # Assuming this is the launch file name in the zed_wrapper package
    )


    # Obstacle detection node
    obstacle_detection_node = Node(
        package='navigation',
        executable='obstacle_detect_node',
        name='obstacle_detection',
        output='screen',
        parameters=[config_file

        #     {
        #     'skip': launch.substitutions.LaunchConfiguration('skip'),
        #     'zed_type': launch.substitutions.LaunchConfiguration('zed_type'),
        #     'display': launch.substitutions.LaunchConfiguration('display')
        # }
        ]
    )

    # Return launch description
    return LaunchDescription([
        # skip_arg,
        # display_arg,
        # zed_type_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(zed_launch_path),
            launch_arguments={
                'camera_name': TextSubstitution(text='zed'),
                'camera_model': TextSubstitution(text='zed2'),  # Specifying zed2 model
            }.items()
        ),
        obstacle_detection_node
    ])
