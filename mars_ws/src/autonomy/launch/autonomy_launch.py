from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import GroupAction, DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Get the params for packages
    aurco_config_file = os.path.join(get_package_share_directory('aruco_detect'),'config', 'DetectorParams.yaml' )
    cam_config_path = os.path.join(get_package_share_directory('start'), 'config', 'cam_config', 'head_cam_params.yaml')
    autonomy_params_file = os.path.join(get_package_share_directory('autonomy'), 'params', 'autonomy_params.yaml')
    hazard_detection_params_dir = os.path.join(get_package_share_directory('hazard_detection', 'params'))

    try:
        # Path to the symlink
        udev_path = '/dev/rover/cameras/autonomyWebCam'

        # Read the symlink to get the relative path it points to (e.g., ../../video8)
        relative_target = os.readlink(udev_path)

        # Extract the final component of the relative path (e.g., 'video8')
        final_device_name = os.path.basename(relative_target)

        # Construct the absolute path in /dev folder (e.g., /dev/video8)
        absolute_target = os.path.join('/dev', final_device_name)

        device = {'video_device': absolute_target}

        # Print the final absolute device path
        print(f"Autonomy Web Cam using: {absolute_target}")
    
    except OSError as e:
        print(f"Error resolving symlink: {e}")
        original_path = '/dev/video2'  # Fallback to a default value if needed
        device = {'video_device': original_path}
    

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('MAPVIZ_LOCATION', default_value='hanksville'),
        
        # USB cam node for the autonomy webcam 
        # NOTE: this will die on a PC docker container because it doesn't have access to your usb devices
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='head_camera',
            namespace ='head_camera',
            parameters=[cam_config_path, device],  # Update this path #TODO: check if this works
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

        # REMOVED from Autonomy launch by BRADEN MEYERS FEB 27. LMK if you need it
        # Node(
        #     package='autonomy',
        #     executable='fiducial_data',
        #     name='fiducial_data',
        #     output='screen',
        #     parameters=[autonomy_params_file]
        # ),

        # Removed from autonomy launch by Braden Meyers Apr 26. Will be testing integrated state machine path planner
        # Node(
        #     package='path_planning',
        #     executable='path_planner',
        #     name='path_planner',
        #     output='screen',
        #     parameters=[
        #         {'MAPVIZ_LOCATION': LaunchConfiguration('MAPVIZ_LOCATION')}
        #     ]
        # ),

        # Launch state machine with autonomy namespace
        GroupAction([
            Node(
                package='autonomy',
                executable='state_machine',
                name='state_machine',
                namespace='autonomy',
                output='screen',
                parameters=[autonomy_params_file]
            ),
            # Node( # Launch hazard Detection
            #     package='hazard_detection',
            #     executable='hazard_detector',
            #     name='hazard_detector',
            #     namespace='autonomy',
            #     output='screen',
            #     parameters=[os.path.join(hazard_detection_params_dir, 'hazard_detection.yaml')]
            # )
        ])

    ])


