import rclpy
from rclpy.node import Node
from control_msgs.msg import JointJog
from sensor_msgs.msg import JointState
from rover_msgs.msg import KeyLocations, Elevator, KeyPress


class ArmControlsNode(Node):
    '''
    :author: Nelson Durrant
    :date: November 2024

    ROS2 node that controls the arm and elevator of the rover as it interacts with the keyboard,
    using the detected key locations and the current arm state.

    Subscribes:
        - /key_locations (rovers_msgs/msg/KeyLocations)
        - /arm_state (sensor_msgs/msg/JointState)
    Publishes:
        - /motor_commands (control_msgs/msg/JointJog)
        - /elevator (rovers_msgs/msg/Elevator)
    Services:
        - /key_press (rovers_msgs/srv/KeyPress)
    '''

    def __init__(self):
        '''
        Creates a new ArmControls node.
        '''
        super().__init__('arm_controls')

        self.loc_subscription = self.create_subscription(KeyLocations, '/key_locations', self.loc_listener_callback, 10)
        '''
        Subscription to the "/key_locations" topic with the message type KeyLocations.
        '''
        self.subscription  # Prevent unused variable warning

        self.state_subscription = self.create_subscription(JointState, '/arm_state', self.state_listener_callback, 10)
        '''
        Subscription to the "/arm_state" topic with the message type JointState.
        '''
        self.subscription  # Prevent unused variable warning

        self.cmd_publisher = self.create_publisher(JointJog, '/motor_commands', 10)
        '''
        Publisher to the "/motor_commands" topic with the message type JointJog.
        '''

        self.elevator_publisher = self.create_publisher(Elevator, '/elevator', 10)
        '''
        Publisher to the "/elevator" topic with the message type Elevator.
        '''

        self.srv = self.create_service(KeyPress, '/key_press', self.key_press_callback)
        '''
        Service that attempts to press a certain key based on the KeyPress request.
        '''

    def loc_listener_callback(self, msg):
        '''
        Callback function for the "/key_locations" topic subscription.
        Saves the KeyLocations message to a class variable.

        :param msg: The KeyLocations message received from the "/key_locations" topic.
        '''

        self.curr_key_loc = msg # TODO: Store this correctly

    def state_listener_callback(self, msg):
        '''
        Callback function for the "/arm_state" topic subscription.
        TODO: Add a description of the function.

        :param msg: The JointState message received from the "/arm_state" topic.
        '''

        self.curr_arm_state = msg # TODO: Store this correctly

    def key_press_callback(self, request, response):
        '''
        Callback function for the "/key_press" service.
        Attempts to press the key requested in the KeyPress request.

        :param request: The KeyPress request.
        :param response: The KeyPress response.
        '''

        # TODO: Add fancy controls work here

        response.success = False
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ArmControlsNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()