import rclpy
from rclpy.node import Node
from roboticstoolbox import RevoluteDH, DHRobot
from numpy import pi, array
from utils import Controllable

#################
# MISCELLANEOUS #
#################

N_JOINTS = 5
JOINT_NAMES = [
    "base_joint",  # elevator
    "bracket01_joint",
    "bracket02_joint",
    "bracket03_joint",
    "bracket04_joint",
    "arm05_joint",
]  # used by rviz for simulation
DEG_TO_RAD = pi / 180
INCREMENTS_TO_RAD = pi / 3800  # Convert motor increments to radians
#                                (Maxon motors actuate at 3800 increments per 180 degrees)
CHECK_METHOD_HZ = 10  # How many times /s to call check_method in ModeManager
SUBSCRIBER_QUEUE_SIZE = 1
KD = 0.1  # For non-singularity of JJ.T inversion in pinv IK method
HEARTBEAT_TOLERANCE = 1.0  # How long (sec) since last
#                             heartbeat before stopping the arm
HEARTBEAT_WARMUP = 10.0  # How long to wait on startup before worrying about heartbeat
JBJ_DEFAULT_NODE = Controllable.NODE_5  # Which node to move in joint-by-joint
#                                         if they try to move without selecting one


#########################
# STARTUP CONFIGURATION #
#########################

INITIAL_Q = [0.0, 0.0, 0.0, -pi / 2, 0.0]  # Radians
INITIAL_QDOT = [0.0] * N_JOINTS


################
# JOINT LIMITS #
################

ELEV_VEL_SCALE = 0.00005
VEL_SCALE = 0.025  # (velocity commanded * this + old q) = best guess at new q
SMALL_LIM = 90 * DEG_TO_RAD
MEDIUM_LIM = 140 * DEG_TO_RAD
LARGE_LIM = 160 * DEG_TO_RAD
ELEVATOR_LIM = 1.4  # meters

DEFAULT_QLIM_TOP = array([SMALL_LIM, SMALL_LIM, SMALL_LIM, SMALL_LIM, LARGE_LIM])
DEFAULT_QLIM_BOTTOM = -1 * array(
    [SMALL_LIM, SMALL_LIM, SMALL_LIM, MEDIUM_LIM, LARGE_LIM]
)


##########
# SPEEDS #
##########

SPEEDS = [  # speeds for each joint can be individually modified
    [0.2, 0.25, 0.4, 0.4, 0.4],  # slow
    [0.5, 0.65, 1.0, 1.0, 1.0],  # medium
    [0.7, 0.9, 1.5, 1.5, 1.5],  # fast
]
STARTING_SPEED_INDEX = 1
GRIPPER_SPEED = 250  # Must be an int >= 1
ELEVATOR_SPEEDS = [round(255 * s) for s in [0.5, 0.7, 0.9]]
IK_SPEED_REDUCTION = 0.65

###########################
# PARAMETERIZATION OF ARM #
###########################

# Denavit-Hartenberg parameters for the arm

a1 = 37.8 / 100
a2 = 42.7 / 100
d3 = -5 / 100
alpha3 = -pi / 2
theta4 = -pi / 2
d4 = 7.5 / 100
alpha4 = -pi / 2
d5 = 32.5 / 100

dh_params = [
    RevoluteDH(a=a1),
    RevoluteDH(a=a2),
    RevoluteDH(d=d3, alpha=alpha3),
    RevoluteDH(offset=theta4, d=d4, alpha=alpha4),
    RevoluteDH(d=d5),
]


# ROS 2 Node class to encapsulate this
class ArmParametersNode(Node):
    def __init__(self):
        super().__init__('arm_parameters_node')

        # Declare parameters with default values
        self.declare_parameter('N_JOINTS', N_JOINTS)
        self.declare_parameter('JOINT_NAMES', JOINT_NAMES)
        self.declare_parameter('DEG_TO_RAD', DEG_TO_RAD)
        self.declare_parameter('INCREMENTS_TO_RAD', INCREMENTS_TO_RAD)
        self.declare_parameter('CHECK_METHOD_HZ', CHECK_METHOD_HZ)
        self.declare_parameter('KD', KD)
        self.declare_parameter('HEARTBEAT_TOLERANCE', HEARTBEAT_TOLERANCE)
        self.declare_parameter('HEARTBEAT_WARMUP', HEARTBEAT_WARMUP)
        self.declare_parameter('JBJ_DEFAULT_NODE', JBJ_DEFAULT_NODE)
        self.declare_parameter('INITIAL_Q', INITIAL_Q)
        self.declare_parameter('INITIAL_QDOT', INITIAL_QDOT)

        # More parameters can be declared similarly...

        self.get_logger().info("Arm parameters node has started!")

    def get_parameters(self):
        # Example of retrieving a parameter value
        n_joints = self.get_parameter('N_JOINTS').value
        self.get_logger().info(f"Number of joints: {n_joints}")


def main(args=None):
    rclpy.init(args=args)

    arm_parameters_node = ArmParametersNode()

    # Example of getting a parameter
    arm_parameters_node.get_parameters()

    # For demonstration, plot the arm (robot)
    arm_to_plot = DHRobot(dh_params, name="For demonstration purposes")
    arm_to_plot.plot(INITIAL_Q, block=True)

    rclpy.spin(arm_parameters_node)

    # Shut down ROS 2 node
    arm_parameters_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
