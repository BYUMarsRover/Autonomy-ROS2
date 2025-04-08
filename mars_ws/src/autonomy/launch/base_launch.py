from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os

def generate_launch_description():

    mapviz_location = os.environ.get('MAPVIZ_LOCATION', 'hanksville')
    autonomy_params_file = os.path.join(FindPackageShare('autonomy'), 'params', 'autonomy_params.yaml')

    autonomy_gui_launch = Node(
        package='autonomy',
        executable='autonomy_gui',
        name='autonomy_gui',
        output='screen',
        parameters=[
            autonomy_params_file,
            {'MAPVIZ_LOCATION': LaunchConfiguration('MAPVIZ_LOCATION')}
        ]
    )

    return LaunchDescription([
        mapviz_location,
        autonomy_params_file,
        autonomy_gui_launch
    ])
