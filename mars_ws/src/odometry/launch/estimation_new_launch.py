from launch import LaunchDescription
from launch.actions import GroupAction, DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # config = os.path.join(get_package_share_directory('odometry'), 'config', 'estimation.yaml'),
    imu_config = os.path.join(get_package_share_directory('odometry'), 'config', 'imu_filter.yaml'),

    return LaunchDescription([
        # Load parameters for robot_localization


        Node(
            package='odometry',
            executable='rover_state_singleton_creator_new',
            name='rover_state_singleton_creator_new',
            output='screen'
        ),

        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_madgwick',
            output='screen',
            remappings=[
                ('imu/data_raw', 'zed/zed_node/imu/data'),
                ('imu/mag', 'zed/zed_node/imu/mag')
            ],
            parameters=[imu_config]
        ),
    ])