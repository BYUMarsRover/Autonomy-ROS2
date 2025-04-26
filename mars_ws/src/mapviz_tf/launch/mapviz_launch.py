from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, LogInfo, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from pathlib import Path


def generate_launch_description():
    # Mapviz_config
    home_path = os.getenv('HOME', '/home/marsrover')
    config_path = os.path.join(home_path, 'mars_ws/src/mapviz_tf', 'scripts', '.mapviz_config')
    # Mapviz origins param file
    mapviz_origins_path = os.path.join(
        home_path, 'mars_ws/src/mapviz_tf/params/mapviz_origins.yaml'
    )
    
    mapviz_origins = Path(mapviz_origins_path).read_text()
  
    return LaunchDescription([
        # Set environment variable
        SetEnvironmentVariable('ROSCONSOLE_FORMAT', '[${thread}] [${node}/${function}:${line}]: ${message}'),

        # Declare launch arguments
        DeclareLaunchArgument('print_profile_data', default_value='false'),
        DeclareLaunchArgument('MAPVIZ_LOCATION', default_value='hanksville'),

        LogInfo(msg=['Config file path: ', config_path]),

        # Node for mapviz
        Node(
            package='mapviz',
            executable='mapviz',
            name='mapviz',
            parameters=[{
                'print_profile_data': LaunchConfiguration('print_profile_data'),
                'config': config_path,
            }],
        ),

        # Node for initialize_origin
        Node(
            package='swri_transform_util',
            executable='initialize_origin.py',
            name='initialize_origin',
            remappings=[('/fix', '/ins/lla')],
            parameters=[
                {'local_xy_frame': '/map'},
                {'local_xy_origin': LaunchConfiguration('MAPVIZ_LOCATION')},
                {'local_xy_origins': mapviz_origins},
            ]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='swri_transform',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'origin'],
            parameters=[
                {"local_xy_frame": "map"},
                {"local_xy_origin": "auto"},
                {"local_xy_navsatfix_topic": "/gps/fix"}
            ]
        ),
        Node(
            package='mapviz_tf', 
            executable='rover_tf_broadcaster',
            name='rover_tf_broadcaster',
            output='log',
            parameters=[
                {'MAPVIZ_LOCATION': LaunchConfiguration('MAPVIZ_LOCATION')},
            ],
        ),

        # Other nodes
        # Node(package='mapviz_tf', executable='path_to_mapviz', name='path_to_mapviz', output='log'),
        # Node(package='mapviz_/tf', executable='gps_to_mapviz', name='gps_to_mapviz', output='log'),
        # Node(package='mapviz_tf', executable='click_waypoint', name='waypoint_picker', output='log'),
        # Node(package='rosapi', executable='rosapi_node', name='rosapi', output='log'),
    ])
