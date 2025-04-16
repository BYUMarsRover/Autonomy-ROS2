from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        
        # Launch the joy_node (renamed to xbox_node_drive)
        # The joy_node_linux is used instead of the joy_node to use the 'dev' parameter for symbolic links
        Node(
            package='joy_linux',
            executable='joy_linux_node',
            name='xbox_node_science',
            parameters=[{'dev': '/dev/rover/js/xbox_360_science'}],
            remappings=[('/joy', '/joy_science_input')]
        ),

        # Launch the custom xbox_science node
        Node(
            package='joysticks',
            executable='xbox_science',
            name='xbox_science',
            output='screen'
        ),
    ])
