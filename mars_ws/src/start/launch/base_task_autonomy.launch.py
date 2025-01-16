
# ==================================
# ==== Base Autonomy Task Launch====
# ==================================

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # import environment variables
    mapviz_location=os.environ.get('MAPVIZ_LOCATION', '')

    # Start all common launch files on the base station
    include_base_common = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('start'),
                'launch',
                'base_common.launch.py'
            )
        ])
    )

    # Start launch files specific to the Autonomy Task on the base station
    # include_base_autonomous = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         os.path.join(
    #             get_package_share_directory('navigation'),
    #             'launch',
    #             'base_autonomous.launch.py'
    #         )
    #     ])
    # )

    # include_path_planning = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         os.path.join(
    #             get_package_share_directory('path_planning'),
    #             'launch',
    #             'path_planning_state_machine.launch.py'
    #         )
    #     ])
    # )

    return LaunchDescription([
        DeclareLaunchArgument('MAPVIZ_LOCATION', default_value=mapviz_location),
        include_base_common,
        # include_base_autonomous,
        # include_path_planning
    ])