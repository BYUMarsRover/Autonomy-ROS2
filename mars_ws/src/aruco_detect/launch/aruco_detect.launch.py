from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Define launch arguments
    return LaunchDescription([
        DeclareLaunchArgument('camera', default_value='/usb_cam', description='Namespace for camera input'),
        DeclareLaunchArgument('image', default_value='image_raw', description='Image topic name'),
        DeclareLaunchArgument('transport', default_value='compressed', description='Image transport type'),
        DeclareLaunchArgument('fiducial_len', default_value='0.2', description='Fiducial length in meters'),
        DeclareLaunchArgument('dictionary', default_value='DICT_4X4_50', description='Dictionary ID for ArUco detection'),
        DeclareLaunchArgument('do_pose_estimation', default_value='true', description='Enable pose estimation'),
        DeclareLaunchArgument('vis_msgs', default_value='false', description='Use ROS standard vision_msgs for pose estimation'),
        DeclareLaunchArgument('ignore_fiducials', default_value='', description='List of fiducials to ignore'),
        DeclareLaunchArgument('fiducial_len_override', default_value='', description='Override fiducial length'),
        DeclareLaunchArgument('verbose', default_value='false', description='Enable verbose output'),
        DeclareLaunchArgument('node_name', default_value='aruco_detect', description='Name of the node'),

        # Node configuration
        Node(
            package='aruco_detect',
            executable='aruco_detect',
            name=LaunchConfiguration('node_name'),
            output=['screen'],
            parameters=[{
                'image_transport': LaunchConfiguration('transport'),
                'publish_images': True,
                'fiducial_len': LaunchConfiguration('fiducial_len'),
                'dictionary': LaunchConfiguration('dictionary'),
                'do_pose_estimation': LaunchConfiguration('do_pose_estimation'),
                'vis_msgs': LaunchConfiguration('vis_msgs'),
                'ignore_fiducials': LaunchConfiguration('ignore_fiducials'),
                'fiducial_len_override': LaunchConfiguration('fiducial_len_override'),
                'verbose': LaunchConfiguration('verbose'),
            }],
            remappings=[
                ('camera/compressed', [
                    LaunchConfiguration('camera'), '/', LaunchConfiguration('image'), '/', LaunchConfiguration('transport')
                ]),
                ('camera_info', [LaunchConfiguration('camera'), '/camera_info']),
                ('/fiducial_transforms', [LaunchConfiguration('node_name'), '/fiducial_transforms']),
                ('/fiducial_vertices', [LaunchConfiguration('node_name'), '/fiducial_vertices']),
                ('/fiducial_data', [LaunchConfiguration('node_name'), '/fiducial_data']),
                ('/fiducial_images', [LaunchConfiguration('node_name'), '/fiducial_images']),
            ],
            respawn=True,
            respawn_delay=5
        ),
    ])
