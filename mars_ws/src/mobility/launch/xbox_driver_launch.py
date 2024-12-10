from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetParameter
from launch_ros.actions import Node


def generate_launch_description():
    # Declare the parameter for the device
    declare_xbox_device_param = DeclareLaunchArgument(
        'xbox_device',
        default_value='/dev/rover/js/xbox_one',
        description='Path to the Xbox joystick device'
    )
    namespace = 'mobility'

    return LaunchDescription([
        declare_xbox_device_param,

        # Set the parameter for the Xbox device
        SetParameter(
            name='xbox_node_drive/dev',
            value='/dev/rover/js/xbox_one'
        ),

        # Xbox Node Drive (joy_node)
        Node(
            package='joy',
            executable='joy_node',
            name='xbox_node_drive',
            remappings=[('/joy', '/joy_drive_input')],
            output='screen',
            namespace=namespace,
        ),

        # Xbox Drive Node in the "mobility" namespace
        Node(
            package='mobility',
            executable='joystick_control.py',
            name='xbox_drive',
            namespace='mobility',
            output='screen',
            namespace=namespace,
        ),
    ])
