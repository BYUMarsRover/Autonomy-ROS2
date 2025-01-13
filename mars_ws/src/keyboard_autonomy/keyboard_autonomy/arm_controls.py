import rclpy
from rclpy.node import Node
from control_msgs.msg import JointJog
from sensor_msgs.msg import JointState
from roboticstoolbox import DHRobot
from rover_msgs.msg import KeyLocations, Elevator
from rover_msgs.srv import KeyPress
import numpy as np
import parameters as p

L = np.array([[0.31],[0.34],[0.08]]) # arm lengths (meters), upper segment to lower segment

### Controller Constants ###

DESIRED_POS = [0, 0] # TODO: Add desired key position in the camera frame
CLOSE = 5 # TODO: Add buffer for how close the key needs to be to the desired position
STABLE_REQ = 3 # TODO: Add number of frames the key needs to be in the desired position

ARM_BASE = 0.0 # TODO: Add base position of arm, do we need this?

ELEV_KP = 0.01 # TODO: Tune kp value for elevator
ARM_KP = 0.01 # TODO: Tune kp value for arm


class ArmControlsNode(Node):
    '''
    :author: Nelson Durrant, Sarah Sanderson
    :date: November 2024

    ROS2 node that controls the arm and elevator of the rover as it interacts with the keyboard,
    using the detected and desired key locations.

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
        self.elevator_subsription = self.create_subscription(Elevator, "/elevator", self.update_elev, 10)
        '''
        Subscription to the "/elevator" topic with the message type Elevator
        '''

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

        # Arm DH model for IK
        self.arm_dh_model = DHRobot(p.dh_params, name="arm")

        # Initial conditions
        self.arm_dh_model.q = np.array(p.INITIAL_Q)

        self.get_logger().info("ArmControlsNode started")

    def update_elev(self, elevator_cmd: Elevator):
        """
        Updates the elevator position. This is just a rough estiate based on relative position and speed
        """
        #TODO: Needs tunning
        dir = 1 if elevator_cmd.elevator_direction else -1
        speed = dir * elevator_cmd.elevator_speed
        if (dir == 1 and self.arm_dh_model.q[0] + p.ELEV_PWM_TO_VEL * speed < p.TOP_ELEVATOR_LIM):
            if (dir == -1 and self.arm_dh_model.q[0] + p.ELEV_PWM_TO_VEL * speed > p.BOTTOM_ELEVATOR_LIM):
                self.arm_dh_model.q[0] += p.ELEV_PWM_TO_VEL * speed

    def update_pos(self, measured_joint_pos: JointState):
        """
        Upon receiving a joint position measured by the Hall effect sensors
        internal to the motors, uses this as new estimate of joint positions
        Does not update for the elevator
        """
        # The indexing is just some defensive programming
        self.arm_dh_model.q[1:] = np.array(measured_joint_pos.position[-(self.arm_dh_model.n-1) :]) ## still need to fix this for elevator

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
            arm_msg = JointJog()
            # TODO: needs to have the homography matrix passed in
            H = np.array([[0, 0, 0],
                         [0, 0, 0],
                         [0, 0, 0]])
            # TODO: Add arm control

            lambda_v = 1 
            lambda_omega = 1

            K = np.array([[240.0742270720785, 0.0, 303.87271958823317], 
                          [0.0, 240.2956790772891, 242.63742883611513],
                          [0.0, 0.0, 1.0]], dtype=np.float32)
            # Calculate e_v and e_omega
            p_star = np.array([K[0][2], K[1][2], 1])
            m_star = np.linalg.inv(K) @ p_star

            e_v = (H - np.eye(3)) @ m_star

            e_omega_skew = H - H.transpose()
            e_omega = np.array([e_omega_skew[2][1], e_omega_skew[0][2], e_omega_skew[1][0]])

            # Compute the twist in the camera frame
            # Twist is a 6x1 matrix that has [0:3] as the linear velocity and [3:end] being the rotation velocity needed to command the camera frame to match the desired homography
            twist = -np.vstack(np.hstack(lambda_v*np.eye(3), np.zeros((3,3))), np.hstack(np.zeros((3,3)), lambda_omega*np.eye(3))) @ np.vstack(e_v.transpose(), e_omega.transpose())

            # Tranform the twist from the camera frame into the base frame
            transform_matrix = self.arm_dh_model.fkine(self.arm_dh_model.q)
            rotation_matrix = np.array([transform_matrix[0][:3], transform_matrix[1][:3], transform_matrix[2][:3]])
            translation = np.array(transform_matrix[:3][3])
            translation = translation.flatten()
            skew = np.array([[0, -translation[2], translation[1]],
                             [translation[2], 0, -translation[0]],
                             [-translation[1], translation[0], 0]])
            Z_shift = np.vstack(np.hstack(np.eye(3), -skew), np.hstack(np.zeros((3, 3)), np.eye(3))) @ np.vstack(np.hstack(rotation_matrix, np.zeros((3,3))), np.hstack(np.zeros((3,3)), rotation_matrix))
            twist = Z_shift @ twist
            # Find the jacobian and take the psuedo-inverse and multiply the new twist
            J = self.arm_dh_model.jacob0(self.arm_dh_model.q)
            J_dagger = J.T @ np.inv(J @ J.T + p.KD**2 * np.eye(len(J))) # pseudo-inverse IK method

            qdot = J_dagger @ twist
            qdot = qdot.flatten()

            # Take the q_dot and send it to the coresponding motors
            elevPWM = round(qdot[0] / p.ELEV_PWM_TO_VEL)

            ## Publish the arm joint commands
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
    
### Arm Forward and Inverse Kinematic Functions from Sarah ###

# Forward Kinematics (Joint Angles to Hand Position)
# x = [x;
#      y]
def q2x(q):
    x = np.zeros((2,np.size(q,1)))

    if np.size(q,0) == 2:
        x[0,:] = L[0]*np.cos(q[0,:]) + L[1]*np.cos(q[0,:]+q[1,:])
        x[1,:] = L[0]*np.sin(q[0,:]) + L[1]*np.sin(q[0,:]+q[1,:])
    else:
        x[0,:] = L[0]*np.cos(q[0,:]) + L[1]*np.cos(q[0,:]+q[1,:]) + L[2]*np.cos(q[0,:]+q[1,:]+q[2,:])
        x[1,:] = L[0]*np.sin(q[0,:]) + L[1]*np.sin(q[0,:]+q[1,:]) + L[2]*np.sin(q[0,:]+q[1,:]+q[2,:])

    return x


# Inverse Kinematics (Hand Position to Joint Angles)
# **MAKE SURE TO CONVERT Q TO RADS
# q = [q_shoulder;
#      q_elbow    ]
def x2q(x):
    if np.size(x) == 2:
        q = np.zeros((np.size(x),1))
        q[1] = np.arccos((x[0]**2 + x[1]**2 - L[0]**2 - L[1]**2) / (2*L[0]*L[1]))
        beta = np.arccos((x[0]**2 + x[1]**2 + L[0]**2 - L[1]**2) / (2*L[0]*np.sqrt(x[0]**2 + x[1]**2)))
        alpha = np.arctan2(x[1],x[0])
        q[0] = alpha - beta
    else:
        q = np.zeros(np.size(x))
        q[1,:] = np.arccos((x[0,:]**2 + x[1,:]**2 - L[0]**2 - L[1]**2) / (2*L[0]*L[1]))
        beta = np.arccos((x[0,:]**2 + x[1,:]**2 + L[0]**2 - L[1]**2) / (2*L[0]*np.sqrt(x[0,:]**2 + x[1,:]**2)))
        alpha = np.arctan2(x[1,:],x[0,:])
        q[0,:] = alpha - beta

    return q


# Minimum Jerk Trajectory (Determines the smoothest path between two points)
# w_i: initial position
# w_f: final position
# T: movement duration (scalar)
# t: time vector (usually np.arange(0,T,0.001))
# w: output trajectory
# dw: output velocity
# dw2: output acceleration
# dw3: output jerk
def min_jerk(w_i,w_f,T,t):
    if type(t) == float:
        w = np.zeros((len(w_i),1))
        dw = np.zeros((len(w_i),1))
        dw2 = np.zeros((len(w_i),1))
        dw3 = np.zeros((len(w_i),1))
    else:
        w = np.zeros((len(w_i),len(t)))
        dw = np.zeros((len(w_i),len(t)))
        dw2 = np.zeros((len(w_i),len(t)))
        dw3 = np.zeros((len(w_i),len(t)))

    for i in range(len(w_i)):
        # Position
        w[i,:] = w_i[i] + (w_f[i] - w_i[i])*(10*(t/T)**3 - 15*(t/T)**4 + 6*(t/T)**5)
        # Velocity
        dw[i,:] = 30*(w_f[i] - w_i[i])*((t/T)**2 - 2*(t/T)**3 + (t/T)**4)*(1/T)
        # Acceleration
        dw2[i,:] = 60*(w_f[i] - w_i[i])*((t/T)-3*(t/T)**2+2*(t/T)**3)*T**(-2)
        # Jerk
        dw3[i,:] = 60*(w_f[i] - w_i[i])*(1 - 6*(t/T) + 6*(t/T)**2)*T**(-3)

    return w, dw, dw2, dw3


def main(args=None):
    rclpy.init(args=args)
    node = ArmControlsNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()