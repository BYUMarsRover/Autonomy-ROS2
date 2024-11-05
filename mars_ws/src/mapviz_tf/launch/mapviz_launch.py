from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os

mapviz_tf_dir = get_package_share_directory('mapviz_tf')


def generate_launch_description():
    return LaunchDescription([
        # Set environment variable
        SetEnvironmentVariable('ROSCONSOLE_FORMAT', '[${thread}] [${node}/${function}:${line}]: ${message}'),

        # Declare launch arguments
        DeclareLaunchArgument('print_profile_data', default_value='false'),
        DeclareLaunchArgument('location', default_value=LaunchConfiguration('MAPVIZ_LOCATION', default='hanksville')),

        # Node for mapviz
        Node(
            package='mapviz',
            executable='mapviz',
            name='mapviz',
            parameters=[{
                'print_profile_data': LaunchConfiguration('print_profile_data'),
                'config': os.path.join(mapviz_tf_dir, '/scripts/.mapviz_config'),
            }]
        ),

        # Conditional parameters for map_origin_index
        Node(
            package='swri_transform_util',
            executable='initialize_origin.py',
            name='initialize_origin',
            output='screen',
            parameters=['/home/user/BYU-Mars-Rover/rover_ws/src/mapviz_tf/launch/local_xy_origins.yaml']
        ),

        # Static transform publisher
        Node(
            package='tf',
            executable='static_transform_publisher',
            name='swri_transform',
            parameters=[{
                'args': '0 0 0 0 0 0 /map /origin 100'
            }]
        ),

        # Other nodes
        Node(package='mapviz_tf', executable='rover_tf_broadcaster.py', name='rover_tf_broadcaster', output='screen'),
        Node(package='mapviz_tf', executable='path_to_mapviz.py', name='path_to_mapviz', output='screen'),
        Node(package='mapviz_tf', executable='gps_to_mapviz.py', name='gps_to_mapviz', output='screen'),
        Node(package='mapviz_tf', executable='click_waypoint.py', name='waypoint_picker', output='screen'),
        Node(package='rosapi', executable='rosapi_node', name='rosapi', output='screen'),
    ])
