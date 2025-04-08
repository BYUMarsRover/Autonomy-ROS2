from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # Launch the joy_node (renamed to xbox_node_drive)
        # The joy_node_linux is used instead of the joy_node to use the 'dev' parameter for symbolic links
        Node(
            package='joy_linux',
            executable='joy_linux_node',
            name='xbox_node_drive',
            parameters=[{'dev': '/dev/rover/js/xbox_one_drive'}],
            remappings=[('/joy', '/joy_drive_input')]
        ),

        # Launch the xbox_drive node
        Node(
            package='joysticks',
            executable='xbox_drive',
            name='xbox_drive',
            # output='screen'
        )
    ])
