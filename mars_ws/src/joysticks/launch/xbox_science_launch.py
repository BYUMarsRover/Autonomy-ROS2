import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Declare the parameter for the xbox node
        DeclareLaunchArgument(
            'xbox_dev',
             default_value='/dev/rover/js/xbox_one', 
             description='Xbox controller device path'
             ),

        # Launch the joy node (remap "/joy" to "/joy_science_input")
        Node(
            package='joy',
            executable='joy_node',
            name='xbox_node_science',
            remappings=[('/joy', '/joy_science_input')],
            parameters=[{'dev': LaunchConfiguration('xbox_dev')}]
        ),

        # Launch the custom xbox_science node
        Node(
            package='joysticks',
            executable='xbox_science',
            name='xbox_science',
            output='screen'
        ),
    ])
