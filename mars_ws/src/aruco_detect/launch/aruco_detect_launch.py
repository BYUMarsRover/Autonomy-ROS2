from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_file = os.path.join(get_package_share_directory('aruco_detect'),'config', 'DetectorParams.yaml' )
    # Define launch arguments
    return LaunchDescription([

        # Node configuration
        Node(
            package='aruco_detect',
            executable='aruco_detect',
            name='aruco_detect_logi',
            output=['screen'],
            parameters=[config_file],
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
    ])
