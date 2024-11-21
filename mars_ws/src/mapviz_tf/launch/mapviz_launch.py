from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, LogInfo
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
from launch.conditions import IfCondition
import os

def generate_launch_description():
    config_path = os.path.join(
        os.getenv('HOME', '/home/marsrover'),  # Fallback in case HOME is not set
        '/mars_ws/src/mapviz_tf/scripts/.mapviz_config'     # removed marsrover/ at the beginning
    )
    yaml_path = os.path.join(
    os.getenv('HOME', '/home/marsrover'),
    'mars_ws/src/mapviz_tf/params/mapviz_params.yaml'
    )

    return LaunchDescription([
        # Set environment variable
        SetEnvironmentVariable('ROSCONSOLE_FORMAT', '[${thread}] [${node}/${function}:${line}]: ${message}'),

        # Declare launch arguments
        DeclareLaunchArgument('print_profile_data', default_value='false'),
        DeclareLaunchArgument('location', default_value=LaunchConfiguration('MAPVIZ_LOCATION', default='hanksville')),

        LogInfo(msg=['Config file path: ', config_path]),

        # Define the dictionary of origins
        local_xy_origins = {
          'byu': {
              'latitude': 40.2497218,
              'longitude': -111.649276,
              'altitude': 1376.0,
              'heading': 0.0,
          },
          'rock_canyon': {
              'latitude': 41.267147,
              'longitude': -111.632455,
              'altitude': 0.0,
              'heading': 0.0,
          },
          'hanksville': {
              'latitude': 38.406441,
              'longitude': -110.791932,
              'altitude': 1375.0,
              'heading': 0.0,
          },
          'gravel_pit': {  
              'latitude': 40.322243,
              'longitude': -111.644278,
              'altitude': 1500.0,
              'heading': 0.0,
          },
          'little_moab': {  
              'latitude': 40.057020,
              'longitude': -112.012014,
              'altitude': 1500.0,
              'heading': 0.0,
          },
        }

        # A function to resolve which dictionary entry to use
        def get_origin_params(context):
            origin_key = LaunchConfiguration('location').perform(context)
            return local_xy_origins.get(origin_key, {})  # Default to an empty dict if key is not found

        # Node for mapviz
        Node(
            package='mapviz',
            executable='mapviz',
            name='mapviz',
            parameters=[{
                'print_profile_data': LaunchConfiguration('print_profile_data'),
                #'config': '$(env HOME)/marsrover/mar_ws/src/mapviz_tf/scripts/.mapviz_config',
            }],
            arguments=['-d', config_path]
        ),

        # Conditional parameters for map_origin_index
        # Node(
        #     package='swri_transform_util',
        #     executable='initialize_origin.py',
        #     name='initialize_origin',
        #     output='screen',
        #     parameters=[yaml_path]
        # ),
        Node(
            package='swri_transform_util',
            executable='initialize_origin.py',
            name='initialize_origin',
            parameters=[{ 
                'local_xy_frame': '/map',
                'local_xy_origin': LaunchConfiguration('local_xy_origin'),
                # Pass the selected origin dictionary dynamically
                'local_xy_origins': get_origin_params
            }]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='swri_transform',
            # arguments=['0', '0', '0', '0', '0', '0', '1', 'wgs84', 'map']
            arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'origin'],
            parameters=[
                {"local_xy_frame" : "map"},
                {"local_xy_origin" : "auto"},
                {"local_xy_navsatfix_topic" : "/gps/fix"}
            ]
        ),

        # Other nodes
        Node(package='mapviz_tf', executable='rover_tf_broadcaster', name='rover_tf_broadcaster', output='screen'),
        Node(package='mapviz_tf', executable='path_to_mapviz', name='path_to_mapviz', output='screen'),
        Node(package='mapviz_tf', executable='gps_to_mapviz', name='gps_to_mapviz', output='screen'),
        Node(package='mapviz_tf', executable='click_waypoint', name='waypoint_picker', output='screen'),
        Node(package='rosapi', executable='rosapi_node', name='rosapi', output='screen'),
    ])
