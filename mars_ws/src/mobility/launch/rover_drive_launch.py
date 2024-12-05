from launch import LaunchDescription
from launch_ros.actions import Node

namespace = 'mobility'
def generate_launch_description():
    # Path to parameter files
    return LaunchDescription([
        # Wheel Manager Node
        Node(
            package='mobility',
            executable='transition.py',
            name='transition',
            output='screen',
            namespace=namespace,
        ),

    ])
