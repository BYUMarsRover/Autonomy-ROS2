from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Define launch arguments
    return LaunchDescription([
        DeclareLaunchArgument(
            'camera', default_value='/zed/zed_node/left', 
            description='Base topic for the camera to use'),
        DeclareLaunchArgument(
            'image', default_value='image_rect_color', 
            description='The image topic name'),
        DeclareLaunchArgument(
            'transport', default_value='compressed', 
            description='Transport type to be used by image_transport'),
        DeclareLaunchArgument(
            'fiducial_len', default_value='0.2', 
            description='Length of fiducials in meters'),
        DeclareLaunchArgument(
            'dictionary', default_value='0', 
            description='The dictionary ID used by ArUco'),
        DeclareLaunchArgument(
            'do_pose_estimation', default_value='true', 
            description='Enable pose estimation of fiducials'),
        DeclareLaunchArgument(
            'vis_msgs', default_value='false', 
            description='Enable visualization messages'),
        DeclareLaunchArgument(
            'ignore_fiducials', default_value='', 
            description='List of fiducial IDs to ignore'),
        DeclareLaunchArgument(
            'fiducial_len_override', default_value='', 
            description='Override length for specific fiducials'),
        DeclareLaunchArgument(
            'verbose', default_value='false', 
            description='Enable verbose output'),
        DeclareLaunchArgument(
            'node_name', default_value='aruco_detect', 
            description='Name of the ArUco detection node'),

        # Node configuration
        Node(
            package='aruco_detect',
            executable='aruco_detect',
            name=LaunchConfiguration('node_name'),
            output=['screen', 'log'],
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
