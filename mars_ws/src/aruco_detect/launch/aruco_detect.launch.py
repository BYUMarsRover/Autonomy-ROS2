from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get parameter file path
    param_file_path = os.path.join(
        get_package_share_directory('aruco_detect'),
        'config',
        'detector_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='aruco_detect',
            executable='aruco_detect',
            name='aruco_detect',
            output='screen',
            parameters=[param_file_path]
        )
    ])
