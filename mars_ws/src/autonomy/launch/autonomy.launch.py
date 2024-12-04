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
    #usb_cam_launch_dir = os.path.join(get_package_share_directory('start'), 'launch')
    aruco_detect_launch_dir = os.path.join(get_package_share_directory('aruco_detect'), 'launch')
    autonomy_params_dir = os.path.join(get_package_share_directory('autonomy'), 'params')
    hazard_detection_params_dir = os.path.join(get_package_share_directory('hazard_detection', 'params'))

    return LaunchDescription([
        # Declare launch argumensts
        DeclareLaunchArgument('target_ar_tag_id', default_value='-1', description='Target AR Tag ID'),

        #Include autonomy camera launch file (usb_cam) Don't run this on a PC docker computer
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(os.path.join(usb_cam_launch_dir, 'autonomy_camera.launch.py'))
        # ),

        # Include ArUco detection launch for logi webcam
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(aruco_detect_launch_dir, 'aruco_detect.launch.py')),
            launch_arguments={
                'node_name': 'aruco_detect_logi',
                'image': 'image_raw',
                'camera': '/usb_cam'
            }.items(),
        ),

        # Launch state machine, FiducialData, and Hazard Detection with autonomy namespace
        GroupAction([
               Node(
                package='autonomy',
                executable='state_machine',
                name='state_machine',
                namespace='autonomy',
                output='screen',
                parameters=[os.path.join(autonomy_params_dir, 'autonomy_params.yaml')]
            ),
               Node(
                package='autonomy',
                executable='fiducial_data',
                name='fiducial_data',
                namespace='autonomy',
                output='screen',
                parameters=[
                    {'target_ar_tag_id': LaunchConfiguration('target_ar_tag_id')}
                ]
            ),
               Node( # Launch hazard Detection
                package='hazard_detection',
                executable='hazard_detector',
                name='hazard_detector',
                namespace='autonomy',
                output='screen',
                parameters=[os.path.join(hazard_detection_params_dir, 'hazard_detection.yaml')]
            )
        ])
    ])
