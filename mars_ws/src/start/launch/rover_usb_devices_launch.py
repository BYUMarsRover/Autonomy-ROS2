
# ===================================
# ==== Rover USB Devices Launch =====
# ===================================

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os



def generate_launch_description():
    # Start USB devices launch files specific to the Autonomy Task on the rover
    cam_config_path = os.path.join(get_package_share_directory('start'), 'config', 'cam_config', 'head_cam_params.yaml')

    include_mobility = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('mobility'),
                'launch',
                'rover_drive_launch.py'
            )
        ),
    )

    include_gps = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('odometry'),
                'launch',
                'rover_launch.py'
            )
        ),
    )


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
    

    # USB cam node for the autonomy webcam 
        # NOTE: this will die on a Windows PC docker container because it doesn't have access to your usb devices
    include_usb_cam = Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='head_camera',
            namespace ='head_camera',
            parameters=[cam_config_path, device],  # Update this path #TODO: check if this works
            remappings=[
                # ('/image_raw', '/head_camera/image_raw')  # Uncomment and adjust if remapping is needed
            ]
        ),



    return LaunchDescription([
        include_mobility,
        include_gps,
        include_usb_cam
        
    ])