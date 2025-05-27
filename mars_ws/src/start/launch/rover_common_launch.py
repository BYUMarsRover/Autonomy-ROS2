# ===================================
# ====== Rover Commmon Launch =======
# ===================================

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.substitutions import FindPackageShare

# Rover Common Launch
# Launches everything for the rover that is shared among all four tasks.

def generate_launch_description():

    # Environment variable for ROS console output format
    ros_console_format_arg = DeclareLaunchArgument(
        'ROSCONSOLE_FORMAT',
        default_value='(${node})[${severity}]: ${message}',
        description='Console output format'
    )

    rover_address_arg = DeclareLaunchArgument('ROVER_ADDRESS', default_value='192.168.1.120')

    # Mobility
    mobility_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("mobility"), "/launch/rover_drive_launch.py"
        ])
    )

    # Peripherals
    peripheral_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("peripherals"), '/launch/peripherals.launch.py'
        ])
    )

    # GPS
    odometry_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("odometry"), '/launch/rover_launch.py'
        ])
    )

    # Dummy publisher for rover state data when running locally
    dummy_publisher = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("odometry"), '/launch/dummy_singleton_publisher.launch.py'
                ])
            )
        ],
        # Only runs if running locally
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('ROVER_ADDRESS'), "' == '127.0.0.1'"])
        )
    )

    return LaunchDescription([
        ros_console_format_arg,
        rover_address_arg,
        mobility_launch,
        peripheral_launch,
        odometry_launch,
        dummy_publisher
    ])
