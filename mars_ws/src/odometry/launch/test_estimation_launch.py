from launch import LaunchDescription
from launch.actions import GroupAction, DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    config = os.path.join(get_package_share_directory('odometry'), 'config', 'estimation.yaml'),

    return LaunchDescription([
        # Load parameters for robot_localization

        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[config],
            remappings=[
                ('odometry/filtered', 'ekf/odom'),
                ('imu', 'imu/data'),
                ('gps/fix', 'ins/lla'),
            ],
            arguments=['--ros-args', '--log-level', 'info'],
            emulate_tty=True
        ),

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
    ])