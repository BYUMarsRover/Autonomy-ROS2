from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rqt_gui',
            executable='rqt_gui',
            name='rqt_autonomy_gui',
            output='screen',
            respawn=True,
            respawn_delay=2.0
        )
    ])
