import launch
import launch_ros.actions
import launch_ros.descriptions


def generate_launch_description():
    '''
    :author: Nelson Durrant
    :date: November 2024
    
    Launches the keyboard autonomy nodes.

    :return: The launch description.
    '''
    
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='keyboard_autonomy',
            executable='feat2homography.py',
        ),
        launch_ros.actions.Node(
            package='keyboard_autonomy',
            executable='homography2keys.py',
        ),
    ])