from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, LogInfo, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config_path = os.path.join(get_package_share_directory('mapviz_tf'), 'scripts', '.mapviz_config') # NOTE: change if .mapviz_config is moved
    # os.environ['MAPVIZ_CONFIG_PATH'] = config_path

    # yaml_path = os.path.join( # TODO: get the yaml working for info in 'local_xy_origins' NOTE: See mapviz_params.yaml
    #     os.getenv('HOME', '/home/marsrover'),
    #     'mars_ws/src/mapviz_tf/params/mapviz_params.yaml'
    # )

    # Use a default location (or fetch the value dynamically in runtime scripts if necessary)
    current_location = 'byu'  # Change this default as needed
    
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
                'config': config_path,
            }],
        ),

        # Node for initialize_origin
        Node(
            package='swri_transform_util',
            executable='initialize_origin.py',
            name='initialize_origin',
            parameters=[{
                'local_xy_frame': '/map',
                'local_xy_origin': LaunchConfiguration('location'),
                'local_xy_origins': """- name: byu
                                    latitude: 40.2497218
                                    longitude: -111.649276
                                    altitude: 1376.0
                                    heading: 0.0
                                    - name: rock_canyon
                                    latitude: 41.267147
                                    longitude: -111.632455
                                    altitude: 0.0
                                    heading: 0.0
                                    - name: hanksville
                                    latitude: 38.406441
                                    longitude: -110.791932
                                    altitude: 1375.0
                                    heading: 0.0
                                    - name: gravel_pit
                                    latitude: 40.322243
                                    longitude: -111.644278
                                    altitude: 1500.0
                                    heading: 0.0
                                    - name: little_moab
                                    latitude: 40.057020
                                    longitude: -112.012014
                                    altitude: 1500.0
                                    heading: 0.0
                                    """
            }]
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

        # Other nodes
        Node(package='mapviz_tf', executable='rover_tf_broadcaster', name='rover_tf_broadcaster', output='screen'),
        Node(package='mapviz_tf', executable='path_to_mapviz', name='path_to_mapviz', output='screen'),
        Node(package='mapviz_tf', executable='gps_to_mapviz', name='gps_to_mapviz', output='screen'),
        Node(package='mapviz_tf', executable='click_waypoint', name='waypoint_picker', output='screen'),
        Node(package='rosapi', executable='rosapi_node', name='rosapi', output='screen'),
    ])
