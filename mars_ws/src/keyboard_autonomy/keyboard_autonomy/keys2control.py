import rclpy
from rclpy.node import Node
from control_msgs.msg import JointJog
from sensor_msgs.msg import JointState
from rover_msgs.msg import KeyLocations, Elevator

class Keys2ControlNode(Node):
    '''
    :author: Nelson Durrant
    :date: November 2024

    ROS2 node that controls the arm and elevator based on the detected key locations.

    Subscribes:
        - /key_locations (rovers_msgs/msg/KeyLocations)
        - /arm_state (sensor_msgs/msg/JointState)
    Publishes:
        - /motor_commands (control_msgs/msg/JointJog)
        - /elevator (rovers_msgs/msg/Elevator)
    '''

    def __init__(self):
        '''
        Creates a new Keys2Control node.
        '''
        super().__init__('keys2control')

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

        # TODO: Fancy controls work

        cmd_msg = JointJog()
        elevator_msg = Elevator()

        self.cmd_publisher.publish(cmd_msg)
        self.elevator_publisher.publish(elevator_msg)

def main(args=None):
    rclpy.init(args=args)
    node = Keys2ControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()