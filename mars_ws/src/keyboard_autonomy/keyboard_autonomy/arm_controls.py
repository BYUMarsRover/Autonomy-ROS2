import time

import rclpy
from rclpy.node import Node
from control_msgs.msg import JointJog
from sensor_msgs.msg import JointState
from rover_msgs.msg import KeyLocations, Elevator
from rover_msgs.srv import KeyPress


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
        self.get_logger().info("ArmControlsNode started")

        self.key_locations = {
            'a': None,
            'b': None,
            'c': None,
            'd': None,
            'e': [0, 0],
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

        self.state_subscription = self.create_subscription(JointState, '/arm_state', self.state_listener_callback, 10)
        '''
        Subscription to the "/arm_state" topic with the message type JointState.
        '''
        self.state_subscription  # Prevent unused variable warning

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

    def state_listener_callback(self, msg):
        '''
        Callback function for the "/arm_state" topic subscription.
        Saves the JointState message to a class variable.

        :param msg: The JointState message received from the "/arm_state" topic.
        '''

        self.get_logger().info('Received arm state')

        # TODO: It looks like the code is set up so that the arm state is only reported when the 
        # MotorAndSimManager node is running in simulation mode (self.active_connection is False).
        # We'd need to change this in the code, or figure out a way to control the arm based solely
        # on the detected key position from the camera -- which I think should probably be possible,
        # especially if we can get in a good initial position and only need to control one joint or
        # two and the elevator.

        # JOINSTATE MESSAGE FORMAT
        #
        # Header header
        # string[] name
        # float64[] position
        # float64[] velocity
        # float64[] effort

        # JOINT_NAMES FROM MOTORANDSIMMANAGER.PY
        #
        # JOINT_NAMES = [
        # "base_joint",  # elevator
        # "bracket01_joint",
        # "bracket02_joint",
        # "bracket03_joint",
        # "bracket04_joint",
        # "arm05_joint",
        # ]

        # FUNCTION IN MOTORANDSIMMANAGER.PY THAT RUNS ONLY IF SELF.ACTIVE_CONNECTION IS FALSE
        #
        # def publish(self):
        # # Publish normally for controllers
        # self.pub_arm_pos.publish(position=self.joints.position, name=p.JOINT_NAMES[1:])
        # # Publish elevator and reversed positions for RVIZ
        # # For some reason RVIZ has nodes 1-3 rotate backward.
        # RVIZ_pos = [self.elev_pos] + [
        #     pos if i > 2 else -pos for i, pos in enumerate(self.joints.position)
        # ]
        # self.pub_arm_pos_RVIZ.publish(position=RVIZ_pos, name=p.JOINT_NAMES)

        self.base_joint = -1 * msg.position[0]
        self.bracket01_joint = -1 * msg.position[1]
        self.bracket02_joint = -1 * msg.position[2]
        self.bracket03_joint = msg.position[3]
        self.bracket04_joint = msg.position[4]
        self.arm05_joint = msg.position[5]


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
        
        ### Super simple controls for now ###

        DESIRED_POSITION = [0, 0] # Desired key position in the camera frame
        CLOSE_ENOUGH = 10 # Buffer for how close the key needs to be to the desired position
        STABILITY_REQ = 3 # Number of frames the key needs to be in the desired position

        elevator_set = True # TODO: change this
        arm_set = True # TODO: change this
        elevator_stability = 0
        arm_stability = 0

        while not elevator_set and not arm_set:

            # Elevator control
            if DESIRED_POSITION[1] + CLOSE_ENOUGH > self.key_locations[chr(request.key)][1]:
                # TODO: Move elevator up
                elevator_msg = Elevator()
                self.elevator_publisher.publish(elevator_msg)
                elevator_stability = 0
            elif DESIRED_POSITION[1] - CLOSE_ENOUGH < self.key_locations[chr(request.key)][1]:
                # TODO: Move elevator down
                elevator_msg = Elevator()
                self.elevator_publisher.publish(elevator_msg)
                elevator_stability = 0
            else:
                elevator_stability += 1
                if elevator_stability >= STABILITY_REQ:
                    elevator_set = True
                    self.get_logger().info('Elevator stable', once=True)

            # Arm control
            if DESIRED_POSITION[0] + CLOSE_ENOUGH > self.key_locations[chr(request.key)][0]:
                # TODO: Move arm left
                arm_msg = JointJog()
                self.cmd_publisher.publish(arm_msg)
                arm_stability = 0
            elif DESIRED_POSITION[0] - CLOSE_ENOUGH < self.key_locations[chr(request.key)][0]:
                # TODO: Move arm right
                arm_msg = JointJog()
                self.cmd_publisher.publish(arm_msg)
                arm_stability = 0
            else:
                arm_stability += 1
                if arm_stability >= STABILITY_REQ:
                    arm_set = True
                    self.get_logger().info('Arm stable', once=True)

            # TODO: make sure the feature detection is running fast enough for this to work
            # time.sleep(1)

        # TODO: Press the key
        time.sleep(3)

        response.success = True
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ArmControlsNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()