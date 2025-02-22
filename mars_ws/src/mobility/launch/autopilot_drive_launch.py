import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

namespace = 'mobility'
def generate_launch_description():
    # Path to parameter files
    mobility_params = os.path.join(
        get_package_share_directory('mobility'),
        'params',
        'autopilot_params.yaml'
    )

    return LaunchDescription([
        # Wheel Manager Node
        Node(
            package='mobility',
            executable='wheel_manager',
            name='wheel_manager',
            output='screen',
            namespace=namespace
        ),

        # Drive Manager Node
        Node(
            package='mobility',
            executable='drive_manager',
            name='drive_manager',
            output='screen',
            parameters=[mobility_params],
            namespace=namespace
        ),

        # Autopilot Manager Node
        Node(
            package='mobility',
            executable='autopilot_manager',
            name='autopilot_manager',
            output='screen',
            parameters=[mobility_params],
            namespace=namespace
        ),

        # Aruco Autopilot Manager Node
        Node(
            package='mobility',
            executable='aruco_autopilot_manager',
            name='aruco_autopilot_manager',
            output='screen',
            parameters=[mobility_params],
            namespace=namespace
        ),

        # Path Manager Node
        Node(
            package='mobility',
            executable='path_manager',
            name='path_manager',
            output='screen',
            namespace=namespace
        ),
    ])
