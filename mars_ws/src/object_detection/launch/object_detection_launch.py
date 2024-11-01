from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the params folder in your package
    object_detection_dir = get_package_share_directory('object_detection')
    params_file = os.path.join(object_detection_dir, 'params', 'config.yaml')

    return LaunchDescription([
        # Launch the object_detection_node with parameters from the YAML file
        Node(
            package='object_detection',
            executable='object_detection',
            name='object_detection',
            output='screen',
            parameters=[params_file],
        )
    ])
