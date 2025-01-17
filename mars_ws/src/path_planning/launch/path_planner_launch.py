from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    run_path_planner_gui = DeclareLaunchArgument(
        'run_path_planner_gui',
        default_value='false',
        description='Determines whether to run the path planner GUI or not'
    )

    return LaunchDescription([
        run_path_planner_gui,
        Node(
            package='path_planning',
            executable='path_planner',
            name='path_planner',
            output='screen'
        ),
        Node(
            package='path_planning',
            executable='path_planner_gui',
            name='path_planner_gui',
            output='screen',
            condition=IfCondition(LaunchConfiguration('run_path_planner_gui'))
        )
    ])