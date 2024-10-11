from launch import LaunchDescription
import launch
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define launch arguments
    skip_arg = DeclareLaunchArgument('skip', default_value='3')
    display_arg = DeclareLaunchArgument('display', default_value='false')
    zed_type_arg = DeclareLaunchArgument('zed_type', default_value='zed2')

    # ZED Wrapper launch file inclusion
    zed_wrapper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory('zed_wrapper'), '/launch/zed2.launch.py']
        ),
        launch_arguments={'camera_model': 'zed2'}.items()
    )

    # Obstacle detection node
    obstacle_detection_node = Node(
        package='navigation',
        executable='ObstacleDetect.py',
        name='obstacle_detection',
        output='screen',
        parameters=[{
            'skip': launch.substitutions.LaunchConfiguration('skip'),
            'zed_type': launch.substitutions.LaunchConfiguration('zed_type'),
            'display': launch.substitutions.LaunchConfiguration('display')
        }]
    )

    # Return launch description
    return LaunchDescription([
        skip_arg,
        display_arg,
        zed_type_arg,
        zed_wrapper_launch,
        obstacle_detection_node
    ])
