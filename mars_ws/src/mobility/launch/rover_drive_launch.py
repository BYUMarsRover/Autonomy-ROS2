from launch import LaunchDescription
from launch_ros.actions import Node

namespace = 'mobility'
def generate_launch_description():
    # Path to parameter files
    return LaunchDescription([
        # Wheel Manager Node
        Node(
            package='mobility',
            executable='transition',
            name='transition',
            output='screen',
            namespace=namespace,
        )#,
        # Node(
        #    package='mobility',
        #    executable='mega_middleman',
        #    name='mega_middleman',
        #    output='screen',
        #    namespace=namespace,
        # )

    ])
