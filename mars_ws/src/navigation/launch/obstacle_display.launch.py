from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import launch.substitutions

def generate_launch_description():
    # Declare the 'video_size' argument with default value 'small'
    video_size_arg = DeclareLaunchArgument(
        'video_size',
        default_value='small'
    )

    # Define the ObstacleDisplay node
    obstacle_display_node = Node(
        package='navigation',
        executable='ObstacleDisplay.py',
        name='obstacle_display',
        output='screen',
        parameters=[{
            'video_size': launch.substitutions.LaunchConfiguration('video_size')
        }]
    )

    # Return the launch description
    return LaunchDescription([
        video_size_arg,
        obstacle_display_node
    ])
