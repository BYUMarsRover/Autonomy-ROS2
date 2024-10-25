import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('ROSCONSOLE_FORMAT', '[${thread}] [${node}/${function}:${line}]: ${message}'),

        DeclareLaunchArgument('print_profile_data', default_value='false'),
        DeclareLaunchArgument('location', default_value=PythonExpression('os.getenv("MAPVIZ_LOCATION", "hanksville")')),

        Node(
            package='mapviz',
            executable='mapviz',
            name='mapviz',
            parameters=[{
                'print_profile_data': LaunchConfiguration('print_profile_data'),
                'config': os.path.join(os.getenv('HOME'), 'BYU-Mars-Rover/rover_ws/src/mapviz_tf/scripts/.mapviz_config')
            }]
        ),

        Node(
            package='swri_transform_util',
            executable='initialize_origin.py',
            name='initialize_origin',
            output='screen',
            parameters=[{
                'local_xy_frame': '/map',
                'local_xy_origin': LaunchConfiguration('location'),
                'local_xy_origins': [
                    {'name': 'byu', 'latitude': 40.2497218, 'longitude': -111.649276, 'altitude': 1376.0, 'heading': 0.0},
                    {'name': 'rock_canyon', 'latitude': 40.267147, 'longitude': -111.632455, 'altitude': 0.0, 'heading': 0.0},
                    {'name': 'hanksville', 'latitude': 38.406441, 'longitude': -110.791932, 'altitude': 1375.0, 'heading': 0.0},
                    {'name': 'gravel_pit', 'latitude': 40.322243, 'longitude': -111.644278, 'altitude': 1500.0, 'heading': 0.0},
                    {'name': 'little_moab', 'latitude': 40.057020, 'longitude': -112.012014, 'altitude': 1500.0, 'heading': 0.0},
                ]
            }]
        ),

        Node(
            package='tf',
            executable='static_transform_publisher',
            name='swri_transform',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', '/map', '/origin', '100']
        ),

        Node(package='mapviz_tf', executable='rover_tf_broadcaster.py', name='rover_tf_broadcaster', output='screen'),
        Node(package='mapviz_tf', executable='path_to_mapviz.py', name='path_to_mapviz', output='screen'),
        Node(package='mapviz_tf', executable='gps_to_mapviz.py', name='gps_to_mapviz', output='screen'),
        Node(package='mapviz_tf', executable='click_waypoint.py', name='waypoint_picker', output='screen'),
        Node(package='rosapi', executable='rosapi_node', name='rosapi')
    ])