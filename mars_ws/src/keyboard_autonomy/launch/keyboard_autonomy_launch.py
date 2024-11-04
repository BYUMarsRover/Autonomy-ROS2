#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    # Declare arguments
    simulation_arg = DeclareLaunchArgument('simulation', default_value='false')
    target_ar_tag_id_arg = DeclareLaunchArgument('target_ar_tag_id', default_value='-1')
    display_arg = DeclareLaunchArgument('display', default_value='false')
    depth_arg = DeclareLaunchArgument('depth', default_value='true')
    video_res_arg = DeclareLaunchArgument('video_res', default_value='1080')
    camera_arg = DeclareLaunchArgument('camera', default_value='zed2i')

    # Parameters
    simulation = LaunchConfiguration('simulation')
    target_ar_tag_id = LaunchConfiguration('target_ar_tag_id')
    camera = LaunchConfiguration('camera')

    # Remappings for simulation
    simulation_group = GroupAction(
        actions=[
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                remappings=[
                    ('/odometry/filtered', '/ins'),
                    ('/camera/rgb/image_raw', '/zed/image'),
                    ('/camera/depth/image_raw', '/zed/depth_measure'),
                    ('/camera/rgb/camera_info', '/zed/right/camera_info')
                ],
                condition=IfCondition(simulation)
            ),
        ]
    )

    # Remappings and inclusion for non-simulation
    non_simulation_group = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('zed_wrapper'), 'launch', f'{camera}.launch.py')
                ),
                condition=IfCondition(LaunchConfiguration('simulation').perform() == 'false')
            ),
        ]
    )

    # USB camera
    usb_cam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('usb_cam'), 'launch', 'keyboard_camera.launch.py')
        )
    )

    # Keyboard autonomy state machine node
    keyboard_autonomy_group = GroupAction(
        actions=[
            Node(
                package='keyboard_autonomy',
                executable='state_machine_keyboard.py',
                name='state_machine_keyboard_node',
                output='screen',
                parameters=[os.path.join(get_package_share_directory('keyboard_autonomy'), 'params', 'keyboard_autonomy_params.yaml')],
                namespace='keyboard_autonomy'
            )
        ]
    )

    # Debug buffer node
    debug_group = GroupAction(
        actions=[
            Node(
                package='keyboard_autonomy',
                executable='buffer_node.py',
                name='buffer_node',
                output='screen',
                namespace='debug'
            )
        ]
    )

    # Launch description
    ld = LaunchDescription([
        simulation_arg,
        target_ar_tag_id_arg,
        display_arg,
        depth_arg,
        video_res_arg,
        camera_arg,
        simulation_group,
        non_simulation_group,
        usb_cam_launch,
        keyboard_autonomy_group,
        debug_group
    ])

    return ld
