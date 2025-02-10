
# ===================================
# ==== Rover Autonomy Task Launch====
# ===================================

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    # import environment variables
    mapviz_location=os.environ.get('MAPVIZ_LOCATION', '')
    mapviz_location_arg = DeclareLaunchArgument('MAPVIZ_LOCATION', default_value=mapviz_location)

    # Start all common launch files on the rover
    include_rover_common = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('start'),
                'launch',
                'rover_common_launch.py'
            )
        )
    )

    # Start launch files specific to the Autonomy Task on the rover
    include_autonomy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('autonomy'),
                'launch',
                'autonomy_launch.py'
            )
        ),
        launch_arguments={
            'location': LaunchConfiguration('MAPVIZ_LOCATION')
        }.items()
    )

    # TODO: Add when converted to ROS2
    # Start Mobility low level nodes
    include_autopilot_drive = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('mobility'),
                'launch',
                'autopilot_drive_launch.py'
            )
        )
    )

    # XBOX CONTROLS from the rover 
    IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('mobility'), 'launch', 'xbox_drive_launch.py'))
    ),


    include_rover_state_singleton_creator_new = Node(
        package='odometry',
        executable='rover_state_singleton_creator_new',
        name='rover_state_singleton_creator_new',
        output='screen'
    )

    return LaunchDescription([
        mapviz_location_arg,
        include_rover_common,
        include_autonomy,
        include_autopilot_drive,
        include_rover_state_singleton_creator_new
    ])