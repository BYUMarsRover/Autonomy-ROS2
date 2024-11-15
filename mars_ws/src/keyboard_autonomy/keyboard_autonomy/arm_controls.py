import rclpy
from rclpy.node import Node
from control_msgs.msg import JointJog
from rover_msgs.msg import KeyLocations, Elevator
from rover_msgs.srv import KeyPress

### Controller Constants ###

DESIRED_POS = [0, 0] # TODO: Add desired key position in the camera frame
CLOSE = 5 # TODO: Add buffer for how close the key needs to be to the desired position
STABLE_REQ = 3 # TODO: Add number of frames the key needs to be in the desired position

ARM_BASE = 0.0 # TODO: Add base position of arm, do we need this?

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

        # Initial controller states
        self.elev_set = False # Is the elevator in the desired position?
        self.arm_set = False # Is the arm in the desired position?
        self.elev_stability = 0 # How many frames has the elevator been stable?
        self.arm_stability = 0 # How many frames has the arm been stable?
        self.key = None # When this is not None, the controller runs

        self.get_logger().info("ArmControlsNode started")

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

        # Run the controller on receiving new key locations
        if self.key is not None:
            self.control()

    def control(self):

        # Elevator control
        if not elev_set and ((DESIRED_POS[1] + CLOSE > self.key_locations[self.key][1]) or (DESIRED_POS[1] - CLOSE < self.key_locations[self.key][1])):
            # Simple proportional controller
            elev_msg = Elevator()
            # There doesn't seem to be a way to get current elevator position
            elev_msg.elevator_speed = abs(ELEV_KP * (DESIRED_POS[1] - self.key_locations[self.key][1]))
            # 1 to move up, 0 to move down
            elev_msg.elevator_direction = 1 if DESIRED_POS[1] > self.key_locations[self.key][1] else 0
            self.elevator_publisher.publish(elev_msg)
            elev_stability = 0
        else:
            # Ensure the elevator position is stable
            elev_stability += 1
            if elev_stability >= STABLE_REQ:
                elev_set = True
                self.get_logger().info('Elevator stability achieved')

        # Arm control
        if not arm_set and ((DESIRED_POS[0] + CLOSE > self.key_locations[self.key][0]) or (DESIRED_POS[0] - CLOSE < self.key_locations[self.key][0])):
            # Simple proportional controller
            arm_msg = JointJog() # TODO: Get these fields right
            # arm_msg.position = ARM_BASE + ARM_KP * (DESIRED_POS[0] - self.key_locations[self.key][0])
            # self.arm_publisher.publish(arm_msg)
            arm_stability = 0
        else:
            # Ensure the arm position is stable
            arm_stability += 1
            if arm_stability >= STABLE_REQ:
                arm_set = True
                self.get_logger().info('Arm stability achieved')

        if elev_set and arm_set:

            # TODO: Press the button

            self.get_logger().info(f"[SUCCESS] Key {self.key} has been pressed")
            self.key = None # IMPORTANT! This stops the controller

    def key_press_callback(self, request, response):
        '''
        Callback function for the "/key_press" service.
        Attempts to press the key requested in the KeyPress request.

        :param request: The KeyPress request.
        :param response: The KeyPress response.
        '''

        # Check if the feature detector is working
        if self.key_locations[chr(request.key)] is None:
            response.success = False
            self.get_logger().error(f"Key {chr(request.key)} position not found", throttle_duration_sec=2)
            return response
        # Check if the controller is already running
        elif self.key is not None:
            response.success = False
            self.get_logger().info("The controller is running", throttle_duration_sec=5)
            return response
        
        # If none of the above, start the controller for a new key
        self.key = chr(request.key)
        self.get_logger().info(f"Attempting to press key {self.key}")
        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ArmControlsNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()