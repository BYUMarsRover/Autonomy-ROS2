from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen',
            parameters=[{
                'camera_name': 'narrow_stereo',
                'video_device': '/dev/video0', # was '/dev/rover/cameras/autonomyWebCam'
                'image_width': 1920,
                'image_height': 1080,
                'pixel_format': 'yuyv',
                'camera_frame_id': 'usb_cam',
                'io_method': 'mmap',
                'camera_info_url': 'package://start/config/head_camera.yaml' # TODO: correct path
            }],
            respawn=True,
            respawn_delay=5.0
        ),
        Node(
            package='image_view',
            executable='image_view',
            name='image_view',
            output='screen',
            remappings=[('/image', '/autonomy_cam/image_raw')],
            parameters=[{'autosize': True}],
            respawn=True,
            respawn_delay=5.0
        )
    ])