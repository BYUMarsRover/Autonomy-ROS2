from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    config = os.path.join(get_package_share_directory('odometry'), 'config', 'estimation.yaml'),

    return LaunchDescription([
        # Load parameters for robot_localization
        Node(
            package='robot_localization',
            executable='ukf_node',
            name='ukf_se_odom',
            output='screen',
            parameters=[config],
            emulate_tty=True
        ),

        Node(
            package='robot_localization',
            executable='ukf_node',
            name='ukf_se_map',
            output='screen',
            parameters=[config],
            remappings=[
                ('odometry/filtered', 'odometry/filtered_map')
            ],
            emulate_tty=True
        ),

        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[config],
            remappings=[
                ('odometry/filtered', 'odometry/filtered_map')
            ],
            emulate_tty=True
        ),

        Node(
            package='odometry',
            executable='rover_state_singleton_creator',
            name='rover_state_singleton_creator',
            output='screen'
        ),

        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_madgwick',
            output='screen',
            parameters=[{
                'fixed_frame': 'odom',
                'use_mag': True,
                'use_magnetic_field_msg': True,
                'world_frame': 'nwu',
                'publish_tf': False,
                'publish_debug_topics': True
            }]
        ),
    ])