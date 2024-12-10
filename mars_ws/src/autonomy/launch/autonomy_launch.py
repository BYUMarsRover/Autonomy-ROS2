from launch import LaunchDescription
from launch.actions import GroupAction
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Get the params for packages
    aurco_config_file = os.path.join(get_package_share_directory('aruco_detect'),'config', 'DetectorParams.yaml' )
    cam_config_path = os.path.join(get_package_share_directory('start'), 'config', 'cam_config', 'head_cam_params.yaml')
    autonomy_params_file = os.path.join(get_package_share_directory('autonomy'), 'params', 'autonomy_params.yaml')

    return LaunchDescription([
        
        # USB cam node for the autonomy webcam 
        # Include autonomy camera launch file (autonomy) Don't run this on a PC docker computer
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='head_camera',
            namespace='',
            parameters=[cam_config_path],  # Update this path
            remappings=[
                # ('/image_raw', '/head_camera/image_raw')  # Uncomment and adjust if remapping is needed
            ]
        ),

        # Include ArUco detection launch for logi webcam
        Node(
            package='aruco_detect',
            executable='aruco_detect',
            name='aruco_detect_logi',
            output=['screen'],
            parameters=[aurco_config_file],
            remappings=[
                ('camera/compressed', '/usb_cam/image_raw/compressed'),
                ('camera_info', '/usb_cam/camera_info'),
                ('/fiducial_transforms', 'aruco_detect_logi/fiducial_transforms'),
                ('/fiducial_vertices', 'aruco_detect_logi/fiducial_vertices'),
                ('/fiducial_data', 'aruco_detect_logi/fiducial_data'),
                ('/fiducial_images', 'aruco_detect_logi/fiducial_images'),
            ],
            
            #WHAT IS THIS?? 
            respawn=True,
            respawn_delay=5
        ),

        Node(
            package='autonomy',
            executable='fiducial_data',
            name='fiducial_data',
            output='screen',
            parameters=[autonomy_params_file]
        ),

        # Launch state machine with autonomy namespace
        # What is group action and what does it do?
        GroupAction([
               Node(
                package='autonomy',
                executable='state_machine',
                name='state_machine',
                namespace='autonomy',
                output='screen',
                parameters=[autonomy_params_file]
            )
        ])

    ])
