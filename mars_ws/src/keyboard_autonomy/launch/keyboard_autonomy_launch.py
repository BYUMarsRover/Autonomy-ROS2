import sys

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

    # Get the word to type from the command line arguments
    word = "rove"
    for arg in sys.argv:
        if arg.startswith("word:="):
            word = arg.split(":=")[1]

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='keyboard_autonomy',
            executable='feat2homography',
        ),
        launch_ros.actions.Node(
            package='keyboard_autonomy',
            executable='homography2keys',
        ),
        launch_ros.actions.Node(
            package='keyboard_autonomy',
            executable='arm_controls',
        ),
        launch_ros.actions.Node(
            package='keyboard_autonomy',
            executable='keyboard_fsm',
            arguments=[word],
        ),
])