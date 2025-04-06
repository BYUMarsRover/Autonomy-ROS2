from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

namespace = 'mobility'
def generate_launch_description():
    return LaunchDescription([

        # Launch the joy_node (renamed to xbox_node_drive)
        Node(
            package='joy_linux',
            executable='joy_linux_node',
            name='xbox_node_drive',
            parameters=[{'dev': '/dev/rover/js/xbox_one_drive'}],
            remappings=[('/joy', '/joy_drive_input')]
        ),

        # Launch the xbox_drive node
        Node(
            package='mobility',
            executable='joystick',
            name='xbox_drive',
            namespace=namespace,
            # output='screen'
        ),
        Node(
            package='mobility',
            executable='transition',
            name='transition',
            output='screen',
            namespace=namespace,
        ),
        Node(
           package='mobility',
           executable='mega_middleman',
           name='mega_middleman',
           output='screen',
           namespace=namespace,
        )
    ])

