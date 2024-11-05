from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, LogInfo
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
from launch.conditions import IfCondition
import os

def generate_launch_description():
    config_path = os.path.join(
        os.getenv('HOME', '/home/marsrover'),  # Fallback in case HOME is not set
        'marsrover/mar_ws/src/mapviz_tf/scripts/.mapviz_config'
    )
    yaml_path = os.path.join(
        os.getenv('HOME', '/home/marsrover'),  # Fallback in case HOME is not set
        # mars_ws/src/mapviz_tf/launch/local_xy_origins.yaml
        'marsrover/mar_ws/src/mapviz_tf/launch/local_xy_origins.yaml'
    )

    return LaunchDescription([
        # Set environment variable
        SetEnvironmentVariable('ROSCONSOLE_FORMAT', '[${thread}] [${node}/${function}:${line}]: ${message}'),

        # Declare launch arguments
        DeclareLaunchArgument('print_profile_data', default_value='false'),
        DeclareLaunchArgument('location', default_value=LaunchConfiguration('MAPVIZ_LOCATION', default='hanksville')),

        LogInfo(msg=['Config file path: ', config_path]),

        # Node for mapviz
        Node(
            package='mapviz',
            executable='mapviz',
            name='mapviz',
            parameters=[{
                'print_profile_data': LaunchConfiguration('print_profile_data'),
                #'config': '$(env HOME)/marsrover/mar_ws/src/mapviz_tf/scripts/.mapviz_config',
                'config': config_path
            }]
        ),

        # Conditional parameters for map_origin_index
        Node(
            package='swri_transform_util',
            executable='initialize_origin.py',
            name='initialize_origin',
            output='screen',
            parameters=[yaml_path]
        ),

        # Static transform publisher
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='swri_transform',
            parameters=[{
                'args': '0 0 0 0 0 0 /map /origin 100'
            }]
        ),

        # Other nodes
        Node(package='mapviz_tf', executable='rover_tf_broadcaster', name='rover_tf_broadcaster', output='screen'),
        Node(package='mapviz_tf', executable='path_to_mapviz', name='path_to_mapviz', output='screen'),
        Node(package='mapviz_tf', executable='gps_to_mapviz', name='gps_to_mapviz', output='screen'),
        Node(package='mapviz_tf', executable='click_waypoint', name='waypoint_picker', output='screen'),
        Node(package='rosapi', executable='rosapi_node', name='rosapi', output='screen'),
    ])
