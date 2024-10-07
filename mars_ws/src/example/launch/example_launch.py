import launch
import launch_ros.actions
import launch_ros.descriptions


def generate_launch_description():
    
    
    return launch.LaunchDescription([
        
        launch_ros.actions.Node(
            package='example',
            executable='talker'
        ),
        launch_ros.actions.Node(
            package='example',
            executable='listener'
        ),
    ])