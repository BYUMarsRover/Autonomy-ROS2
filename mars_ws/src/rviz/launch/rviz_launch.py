import launch
import launch_ros.actions
import launch_ros.descriptions
from launch_ros.actions import Node

def generate_launch_description():
    
    
    return launch.LaunchDescription([
        
        # launch_ros.actions.Node(
        #     package='rviz',
        #     executable='talker'
        # ),
        # launch_ros.actions.Node(
        #     package='rviz',
        #     executable='listener'
        # ),

        Node(
            package='rviz',
            executable='rviz',
            name='rviz_node',
            output='screen'
        )
    ])