import rclpy
from rclpy.node import Node
from control_msgs.msg import JointJog
from sensor_msgs.msg import JointState
from roboticstoolbox import DHRobot
from rover_msgs.msg import KeyLocations, Elevator, KeyboardHomography
from rover_msgs.srv import KeyPress
import numpy as np
import parameters as p

L = np.array([[0.31], [0.34], [0.08]])  # arm lengths (meters), upper segment to lower segment

### Controller Constants ###

DESIRED_POS = [0, 0]  # TODO: Add desired key position in the camera frame
CLOSE = 5  # TODO: Add buffer for how close the key needs to be to the desired position
STABLE_REQ = 3  # TODO: Add number of frames the key needs to be in the desired position
ERROR_THRESHOLD = 0.1

ARM_BASE = 0.0  # TODO: Add base position of arm, do we need this?

ELEV_KP = 0.01  # TODO: Tune kp value for elevator
ARM_KP = 0.01  # TODO: Tune kp value for arm


class ArmControlsNode(Node):
    '''
    :author: Nelson Durrant, Sarah Sanderson
    :date: November 2024

    ROS2 node that controls the arm and elevator of the rover as it interacts with the keyboard,
    using the detected and desired key locations.

    Subscribes:
        - /key_locations (roversmsgs/msg/KeyLocations)
        - /arm_state (sensor_msgs/msg/JointState)
        - /elevator (rover_msgs/msg/Elevator)
        - /keyboard_homography (rover_msgs/msg/KeyboardHomography)
    Publishes:
        - /motor_commands (control_msgs/msg/JointJog)
        - /elevator (rover_msgs/msg/Elevator)
    Services:
        - /key_press (rover_msgs/srv/KeyPress)
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

        self.arm_pos_subscription = self.create_subscription(JointState, "/arm_state", self.update_pos, 10)
        '''
        Subscription to the "/arm_state" topic with the message type JointState
        '''
        self.elevator_subscription = self.create_subscription(Elevator, "/elevator", self.update_elev, 10)
        '''
        Subscription to the "/elevator" topic with the message type Elevator
        '''
        self.homography_subscription = self.create_subscription(KeyboardHomography, '/keyboard_homography',
                                                                self.update_homography, 10)
        '''
        Subscription to the "/keyboard_homography" topic with the message type KeyboardHomography
        '''
        self.arm_publisher = self.create_publisher(JointJog, '/motor_commands', 10)
        '''
        Publisher to the "/motor_commands" topic with the message type JointJog.
        '''

        self.elevator_publisher = self.create_publisher(Elevator, '/elevator', 10)
        '''
        Publisher to the "/elevator" topic with the message type Elevator.
        '''

        self.srv = self.create_service(KeyPress, '/key_press', self.key_press_callback, 10)
        '''
        Service that attempts to press a certain key based on the KeyPress request.
        '''

        # Initial controller states
        self.elev_set = False  # Is the elevator in the desired position?
        self.arm_set = False  # Is the arm in the desired position?
        self.elev_stability = 0  # How many frames has the elevator been stable?
        self.arm_stability = 0  # How many frames has the arm been stable?
        self.key = None  # When this is not None, the controller runs
        self.homography_matrix = np.eye(3)  # Starting value for homography matrix
        # Arm DH model for IK
        self.arm_dh_model = DHRobot(p.dh_params, name="arm")

        # Initial conditions
        self.arm_dh_model.q = np.array(p.INITIAL_Q)

        self.get_logger().info("ArmControlsNode started")

    def update_elev(self, elevator_cmd: Elevator):
        """
        Updates the elevator position. This is just a rough estimate based on relative position and speed
        """
        # TODO: Needs tuning
        dir = 1 if elevator_cmd.elevator_direction else -1
        speed = dir * elevator_cmd.elevator_speed
        if dir == 1 and self.arm_dh_model.q[0] + p.ELEV_PWM_TO_VEL * speed < p.TOP_ELEVATOR_LIM:
            if dir == -1 and self.arm_dh_model.q[0] + p.ELEV_PWM_TO_VEL * speed > p.BOTTOM_ELEVATOR_LIM:
                self.arm_dh_model.q[0] += p.ELEV_PWM_TO_VEL * speed

    def update_pos(self, measured_joint_pos: JointState):
        """
        Upon receiving a joint position measured by the Hall effect sensors
        internal to the motors, uses this as new estimate of joint positions
        Does not update for the elevator
        """
        # The indexing is just some defensive programming
        self.arm_dh_model.q[1:] = np.array(
            measured_joint_pos.position[-(self.arm_dh_model.n - 1):])  # still need to fix this for elevator

    def update_homography(self, homography: KeyboardHomography):
        """
        Upon receive a message it updates the homography matrix stored
        """
        self.homography_matrix = homography.homography
        self.control()

    def control(self):
        # Arm control
        error = np.linalg.norm(self.homography_matrix - np.eye(3), "fro")
        arm_stability = 0
        if not self.arm_set and error > ERROR_THRESHOLD:
            lambda_v = 1
            lambda_omega = 1

            K = np.array([[240.0742270720785, 0.0, 303.87271958823317],
                          [0.0, 240.2956790772891, 242.63742883611513],
                          [0.0, 0.0, 1.0]], dtype=np.float32)
            # Calculate e_v and e_omega
            p_star = np.array([K[0][2], K[1][2], 1])
            m_star = np.linalg.inv(K) @ p_star

            e_v = (self.homography_matrix - np.eye(3)) @ m_star

            e_omega_skew = self.homography_matrix - self.homography_matrix.transpose()
            e_omega = np.array([e_omega_skew[2][1], e_omega_skew[0][2], e_omega_skew[1][0]])

            # Compute the twist in the camera frame Twist is a 6x1 matrix that has [0:3] as the linear velocity and [
            # 3:end] being the rotation velocity needed to command the camera frame to match the desired homography
            twist = -np.vstack(np.hstack(lambda_v * np.eye(3), np.zeros((3, 3))),
                               np.hstack(np.zeros((3, 3)), lambda_omega * np.eye(3))) @ np.vstack(e_v.transpose(),
                                                                                                  e_omega.transpose())

            # Tranform the twist from the camera frame into the base frame
            transform_matrix = self.arm_dh_model.fkine(self.arm_dh_model.q)
            rotation_matrix = np.array([transform_matrix[0][:3], transform_matrix[1][:3], transform_matrix[2][:3]])
            translation = np.array(transform_matrix[:3][3])
            translation = translation.flatten()
            skew = np.array([[0, -translation[2], translation[1]],
                             [translation[2], 0, -translation[0]],
                             [-translation[1], translation[0], 0]])
            Z_shift = np.vstack(np.hstack(np.eye(3), -skew), np.hstack(np.zeros((3, 3)), np.eye(3))) @ np.vstack(
                np.hstack(rotation_matrix, np.zeros((3, 3))), np.hstack(np.zeros((3, 3)), rotation_matrix))
            twist = Z_shift @ twist
            # Find the jacobian and take the psuedo-inverse and multiply the new twist
            J = self.arm_dh_model.jacob0(self.arm_dh_model.q)
            J_dagger = J.T @ np.inv(J @ J.T + p.KD ** 2 * np.eye(len(J)))  # pseudo-inverse IK method

            qdot = J_dagger @ twist
            qdot = qdot.flatten()

            # Take the q_dot and send it to the corresponding motors
            elevPWM = round(qdot[0] / p.ELEV_PWM_TO_VEL)

            # Publish the arm joint commands
            cmd = JointJog()
            cmd.velocities = [
                qdot[1],
                qdot[2],
                qdot[3],
                qdot[4],
                qdot[5]
            ]
            # Publish joint commands
            self.arm_publisher.publish(cmd)

            # Handle elevator commands
            elevator_cmd = Elevator()
            elevator_cmd.elevator_speed = abs(elevPWM)
            elevator_cmd.elevator_direction = 1 if elevPWM > 0 else 0
            self.elevator_publisher.publish(elevator_cmd)

        else:
            # Ensure the arm position is stable
            arm_stability += 1
            if arm_stability >= STABLE_REQ:
                self.arm_set = True
                self.get_logger().info('Arm stability achieved')

        if self.arm_set:
            # TODO: Press the button

            self.get_logger().info(f"[SUCCESS] Key {self.key} has been pressed")
            self.key = None  # IMPORTANT! This stops the controller

    def key_press_callback(self, request, response):
        if self.arm_set:
            # Simulate pressing the button
            self.key = request.key  # Assuming `key` is a field in your KeyPress request

            # TODO: Add logic to actually press the button if necessary
            self.get_logger().info(f"[SUCCESS] Key {self.key} has been pressed")

            # Respond with success
            response.success = True
            self.key = None  # Reset key state
        else:
            # If arm is not set, log failure and respond
            self.get_logger().warn("Arm is not set. Cannot press the key.")
            response.success = False

        return response

def main(args=None):
    rclpy.init(args=args)
    node = ArmControlsNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
