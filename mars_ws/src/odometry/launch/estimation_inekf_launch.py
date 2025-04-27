from launch import LaunchDescription
from launch.actions import GroupAction, DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    config = os.path.join(get_package_share_directory('odometry'), 'config', 'ekf.yaml'),
    imu_config = os.path.join(get_package_share_directory('odometry'), 'config', 'imu_filter.yaml'),

    return LaunchDescription([
        # Load parameters for robot_localization

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('rover_description'),
                    'launch',
                    'robot_state_publisher.launch.py'
                )
            )
        ),
        Node(
            package='odometry',
            executable='inekf_node',
            name='inekf_node',
            output='screen',
            parameters=[config],
        ),
        


        #Condition to run the singleton creator only on the rover and not
        GroupAction(
            actions=[
                Node(
                    package='odometry',
                    executable='rover_state_singleton_creator',
                    name='rover_state_singleton_creator',
                    output='screen'
                ),
            ],
        ),

        #TODO: Maybe take this out of the other launch file
        Node(
            package='odometry',
            executable='position_velocity_time_translator',
            namespace='rover',
            name='position_velocity_time_translator',
            remappings=[
                ('lla', '/ins/lla')
            ],
            output='screen'
        ),
        
        # Node(
        #     package='imu_filter_madgwick',
        #     executable='imu_filter_madgwick_node',
        #     name='imu_filter_madgwick',
        #     output='screen',
        #     remappings=[
        #         ('imu/data_raw', 'zed/zed_node/imu/data'),
        #         ('imu/mag', 'zed/zed_node/imu/mag')
        #     ],
        #     parameters=[config]
        # ),
    ])