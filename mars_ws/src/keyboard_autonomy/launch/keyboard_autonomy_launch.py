import launch
import launch_ros.actions
import launch_ros.descriptions


def generate_launch_description():
    '''
    :author: Nelson Durrant
    :date: November 2024
    
    Launches the keyboard autonomy node.

    :return: The launch description.
    '''

    # config_file = "/home/frostlab/config/vehicle_config.yaml"
    
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='keyboard_autonomy',
            executable='state_machine_keyboard.py',
            # parameters=[config_file],
        ),
    ])