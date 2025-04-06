from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

namespace = 'mobility'
def generate_launch_description():
    return LaunchDescription([
        # Declare the parameter for xbox_node_drive
        #TODO fix this
        DeclareLaunchArgument(
            'xbox_dev',
            default_value='/dev/rover/js/xbox_one_drive',
            description='Xbox controller device path'
        ),

        # Launch the joy_node (renamed to xbox_node_drive)
        Node(
            package='joy',
            executable='joy_node',
            name='xbox_node_drive',
            parameters=[{'dev': LaunchConfiguration('xbox_dev')}],
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

