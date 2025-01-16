
# ==================================
# ==== Base Autonomy Task Launch====
# ==================================

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Start all common launch files on the base station
    include_base_common = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('start'),
                'launch',
                'base_common_launch.py'
            )
        ])
    )

    # Start launch files specific to the Autonomy Task on the base station
    # (The only thing this does is launch the rqt gui)
    # include_base_autonomous = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         os.path.join(
    #             get_package_share_directory('navigation'),
    #             'launch',
    #             'base_autonomous.launch.py'
    #         )
    #     ])
    # )

    #TODO: in the future, when we have built out path planning, include the launch file here
    
    return LaunchDescription([
        include_base_common,
        # include_base_autonomous,
        # include_path_planning #TODO: uncomment out this when we build out and include path planning
    ])
