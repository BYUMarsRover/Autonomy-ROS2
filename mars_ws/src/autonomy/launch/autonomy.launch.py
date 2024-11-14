from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # Get the share directory for packages
    #zed_wrapper_launch_dir = os.path.join(get_package_share_directory('zed_wrapper'), 'launch')
    usb_cam_launch_dir = os.path.join(get_package_share_directory('start'), 'launch')
    autonomy_launch_dir = os.path.join(get_package_share_directory('autonomy'), 'launch')
    autonomy_params_dir = os.path.join(get_package_share_directory('autonomy'), 'params')
    print(f'Params file path: {os.path.join(autonomy_params_dir, "autonomy_params.yaml")}')

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('target_ar_tag_id', default_value='-1', description='Target AR Tag ID'),
        DeclareLaunchArgument('display', default_value='false', description='Display output'),
        DeclareLaunchArgument('depth', default_value='true', description='Use depth camera'),
        DeclareLaunchArgument('video_res', default_value='1080', description='Video resolution'),
        DeclareLaunchArgument('simulation', default_value='false', description='Run in simulation mode'),

        # Set parameters from arguments
        Node(
            package='autonomy',
            executable='fiducial_data',
            name='fiducial_data',
            output='screen',
            parameters=[
                {'simulation': LaunchConfiguration('simulation')},
                {'target_ar_tag_id': LaunchConfiguration('target_ar_tag_id')}
            ]
        ),

        # Group for ZED wrapper launch if simulation is false
        # GroupAction([
        #     IncludeLaunchDescription(
        #         PythonLaunchDescriptionSource(os.path.join(zed_wrapper_launch_dir, 'zed2i.launch.py'))
        #     )
        # ], condition=IfCondition(LaunchConfiguration('simulation').to_bool() == False)),

        # Include autonomy camera launch file (usb_cam) Don't run this on a PC docker computer
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(os.path.join(usb_cam_launch_dir, 'autonomy_camera.launch.py'))
        # ),

        # Include ArUco detection launch for logi webcam
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(autonomy_launch_dir, 'aruco_detect.launch.py')),
            launch_arguments={
                'node_name': 'aruco_detect_logi',
                'image': 'image_raw',
                'camera': '/usb_cam'
            }.items(),
        ),

        # Launch state machine with autonomy namespace
        GroupAction([
               Node(
                package='autonomy',
                executable='state_machine',
                name='state_machine',
                namespace='autonomy',
                output='screen',
                parameters=[os.path.join(autonomy_params_dir, 'autonomy_params.yaml')]
            )
        ])

    ])
