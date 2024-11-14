import time

import rclpy
from rclpy.node import Node
from control_msgs.msg import JointJog
from sensor_msgs.msg import JointState
from rover_msgs.msg import KeyLocations, Elevator
from rover_msgs.srv import KeyPress

DESIRED_POS = [0, 0] # TODO: Add desired key position in the camera frame
CLOSE = 5 # TODO: Add buffer for how close the key needs to be to the desired position
STABLE_REQ = 3 # TODO: Add number of frames the key needs to be in the desired position

ELEV_BASE = 0.0 # TODO: Add base position of elevator
ARM_BASE = 0.0 # TODO: Add base position of arm

ELEV_KP = 0.01 # TODO: Tune kp value for elevator
ARM_KP = 0.01 # TODO: Tune kp value for arm

class ArmControlsNode(Node):
    '''
    :author: Nelson Durrant
    :date: November 2024

    ROS2 node that controls the arm and elevator of the rover as it interacts with the keyboard,
    using the detected and desired key locations.

    Subscribes:
        - /key_locations (rovers_msgs/msg/KeyLocations)
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
        self.get_logger().info("ArmControlsNode started")

        self.key_locations = {
            'a': None,
            'b': None,
            'c': None,
            'd': None,
            'e': None,
            'f': None,
            'g': None,
            'h': None,
            'i': None,
            'j': None,
            'k': None,
            'l': None,
            'm': None,
            'n': None,
            'o': None,
            'p': None,
            'q': None,
            'r': None,
            's': None,
            't': None,
            'u': None,
            'v': None,
            'w': None,
            'x': None,
            'y': None,
            'z': None,
            'enter': None,
            'caps_lock': None,
            'delete_key': None,
            'space': None
        }

        self.loc_subscription = self.create_subscription(KeyLocations, '/key_locations', self.loc_listener_callback, 10)
        '''
        Subscription to the "/key_locations" topic with the message type KeyLocations.
        '''
        self.loc_subscription  # Prevent unused variable warning

        self.arm_publisher = self.create_publisher(JointJog, '/motor_commands', 10)
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

        # Controller states
        self.elev_set = True # TODO: change this
        self.arm_set = True # TODO: change this
        self.elev_stability = 0
        self.arm_stability = 0
        self.key = None # When this is not None, the controller runs

    def loc_listener_callback(self, msg):
        '''
        Callback function for the "/key_locations" topic subscription.
        Saves the KeyLocations message to a class variable.

        :param msg: The KeyLocations message received from the "/key_locations" topic.
        '''

        self.get_logger().info('Received key locations')
        self.key_locations = {
            'a': msg.a,
            'b': msg.b,
            'c': msg.c,
            'd': msg.d,
            'e': msg.e,
            'f': msg.f,
            'g': msg.g,
            'h': msg.h,
            'i': msg.i,
            'j': msg.j,
            'k': msg.k,
            'l': msg.l,
            'm': msg.m,
            'n': msg.n,
            'o': msg.o,
            'p': msg.p,
            'q': msg.q,
            'r': msg.r,
            's': msg.s,
            't': msg.t,
            'u': msg.u,
            'v': msg.v,
            'w': msg.w,
            'x': msg.x,
            'y': msg.y,
            'z': msg.z,
            'enter': msg.enter,
            'caps_lock': msg.caps_lock,
            'delete_key': msg.delete_key,
            'space': msg.space
        }

        if self.key is not None:
            self.position()

    def position(self):

        # Elevator control
        if (DESIRED_POS[1] + CLOSE > self.key_locations[self.key][1]) or (DESIRED_POS[1] - CLOSE < self.key_locations[self.key][1]):
            # Simple P controller
            elev_msg = Elevator() # TODO: Get request right
            elev_msg.position = ELEV_BASE + ELEV_KP * (DESIRED_POS[1] - self.key_locations[self.key][1])
            self.elevator_publisher.publish(elev_msg)
            elev_stability = 0
        else:
            # Ensure the elevator position is stable
            elev_stability += 1
            if elev_stability >= STABLE_REQ:
                elev_set = True
                self.get_logger().info('Elevator stability achieved', once=True)

        # Arm control
        if (DESIRED_POS[0] + CLOSE > self.key_locations[self.key][0]) or (DESIRED_POS[0] - CLOSE < self.key_locations[self.key][0]):
            # Simple P controller
            arm_msg = JointJog() # TODO: Get request right
            arm_msg.position = ARM_BASE + ARM_KP * (DESIRED_POS[0] - self.key_locations[self.key][0])
            self.arm_publisher.publish(arm_msg)
            arm_stability = 0
        else:
            # Ensure the arm position is stable
            arm_stability += 1
            if arm_stability >= STABLE_REQ:
                arm_set = True
                self.get_logger().info('Arm stability achieved', once=True)

        if elev_set and arm_set:
            # Stop the controller and report correct positioning
            self.positioned = True
            self.key = None


    def key_press_callback(self, request, response):
        '''
        Callback function for the "/key_press" service.
        Attempts to press the key requested in the KeyPress request.

        :param request: The KeyPress request.
        :param response: The KeyPress response.
        '''

        if self.key_locations[chr(request.key)] is None:
            response.success = False
            self.get_logger().warn(f"Key {chr(request.key)} position not found")
            return response

        # Wait for the arm and elevator to be positioned right
        self.positioned = False
        self.key = chr(request.key)
        while not self.positioned:
            time.sleep(1)

        # TODO: Press the button

        response.success = True
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ArmControlsNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()