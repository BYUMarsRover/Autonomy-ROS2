"""
Contains the parameters for the rover arm. The point of this file is to have
    a one-stop-shop for hardcoded values, rather than them being scattered through
    the code.
"""

from roboticstoolbox import RevoluteDH, PrismaticDH, DHRobot
from numpy import pi, array, diag
from keyboard_autonomy.utils import Controllable

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
DEG_TO_RAD = pi / 180.
INCREMENTS_TO_RAD = pi / 3800.  # Convert motor increments to radians
#                                (Maxon motors actuate at 3800 increments per 180 degrees)
CHECK_METHOD_HZ = 10  # How many times /s to call check_method in ModeManager
SUBSCRIBER_QUEUE_SIZE = 1
HEARTBEAT_TOLERANCE = 1.0  # How long (sec) since last
#                             heartbeat before stopping the arm
HEARTBEAT_WARMUP = 10.0  # How long to wait on startup before worrying about heartbeat
JBJ_DEFAULT_NODE = Controllable.NODE_5  # Which node to move in joint-by-joint
#                                         if they try to move without selecting one


#########################
# STARTUP CONFIGURATION #
#########################
# If the initial configuration is changed, the RVIZ will not reflect that.
# I think the urdf file is where the visualization pulls initial values.
# Once a joint update comes on /arm_state, however, RVIZ will update; it will just
# jump from wherever it started to the updated position.

# The canonical position for our arm is straight out in front of the rover,
# with node four hanging down
INITIAL_Q = [0.75, 0.0, 0.0, 0.0, -pi/2, 0.0]  # Radians
INITIAL_QDOT = [0.0] * N_JOINTS
KD = 0.1  # For non-singularity of JJ.T inversion in pinv IK method
Kr = 1.
K = diag([1., 1., 1., Kr, Kr, Kr])


################
# JOINT LIMITS #
################

# Joint limiting constants
ELEV_PWM_TO_VEL = 0.00005 # don't know if this value is correct
VEL_SCALE = 1. #0.025  # this is a magic number that scales down commanded joint velocities and should be removed
SMALL_LIM = 90. * DEG_TO_RAD
MEDIUM_LIM = 140. * DEG_TO_RAD
LARGE_LIM = 160. * DEG_TO_RAD
TOP_ELEVATOR_LIM = 1.0375  # meters
BOTTOM_ELEVATOR_LIM = 0.0625
# Limits. Note that node 4 (index 3) has a lower top limit than bottom
# In some configurations, hitting MEDIUM_LIM on the top will run the arm into itself
DEFAULT_QLIM_TOP = array([SMALL_LIM, SMALL_LIM, SMALL_LIM, 80. * DEG_TO_RAD, LARGE_LIM])
DEFAULT_QLIM_BOTTOM = -1 * array(
    [SMALL_LIM, SMALL_LIM, SMALL_LIM, MEDIUM_LIM, LARGE_LIM]
)


##########
# SPEEDS #
##########

SMOOTHING = .8 # this smooths out the joint velocities so it's not so jerky [1 means only current values, 0 means only last values, .5 means half of both]
RATIO = 75. # the MotorInterface::PVMMove function has legacy code that multiplies joint values by a RATIO of 75, so if simulating, simulate it

# Used to scale qdot in IK controller
# [x (forward backward), y (side to side), z (up down), roll, pitch, yaw]
STARTING_SPEED_INDEX = 0
IK_SPEEDS = [
    [.1, .1, .1, .1, .1, .1], # extra slow
    [.2, .2, .2, .2, .2, .2], # slow
    [.4, .4, .4, .4, .4, .4], # medium
    [.8, .8, .8, .8, .8, .8], # fast
    [1.6, 1.6, 1.6, 1.6, 1.6, 1.6], # ludicrous mode
]

PSEUDO_IK_SPEEDS = [  # speeds for each joint can be individually modified
    [0.2, 0.25, 0.4, 0.4, 0.4],  # slow
    [0.5, 0.65, 1.0, 1.0, 1.0],  # medium
    [0.7, 0.9 , 1.5, 1.5, 1.5],  # fast
]
# IK moves all the joints at once and so feels faster and more likely to break something
# than JBJ, so we reduce the motor speeds in IK by a constant factor
PSEUDO_IK_SPEED_REDUCTION = 0.65

# speeds for each joint can be individually modified
JBJ_SPEEDS = [  
    [0.2, 0.25, 0.4, 0.4, 0.4],  # slow
    [0.5, 0.65, 1.0, 1.0, 1.0],  # medium
    [0.7, 0.9 , 1.5, 1.5, 1.5],  # fast
]
GRIPPER_SPEED = 250  # Must be an int >= 1
# 255 is max for elevator, but last ten percent is non-linear and actually gets slower
ELEVATOR_SPEEDS = [round(255 * s) for s in [0.5, 0.7, 0.9]]
# IK moves all the joints at once and so feels faster and more likely to break something
# than JBJ, so we reduce the motor speeds in IK by a constant factor



###########################
# PARAMETERIZATION OF ARM #
###########################
# according to Denavit-Hartenberg (DH) convention

# Zeroth joint parameters (elevator to arm, added later)
a0 = .0677 # bracket01_joint # bracket_top01
alpha0 = pi # bracket01_joint roll
# First joint parameters
a1 = .372 # bracket02_joint # bracket_bottom02
# Second joint parameters
a2 = .4191 # bracket03_joint # bracket_top03
# Third joint parameters
d3 = .0976 # bracket04_joint z
a3 = .06875 # bracket04_joint x
alpha3 = -pi / 2 # bracket04_joint roll
# Fourth joint parameters
theta4 = pi / 2 # arm05_joint yaw
d4 = .0904 # arm05_joint z
alpha4 = pi / 2 # arm05_joint roll
# Fifth joint parameters
d5 = .14844 # gripper_joint z # gripper_mimic_joint z
alpha5 = pi

dh_params = [
    PrismaticDH(a=a0, alpha=alpha0),
    RevoluteDH(a=a1),
    RevoluteDH(a=a2),
    RevoluteDH(d=d3, a=a3, alpha=alpha3),
    RevoluteDH(offset=theta4, d=d4, alpha=alpha4),
    RevoluteDH(d=d5, alpha=alpha5)
]

# tool frames for IK controller
STARTING_FRAME_INDEX = 2
TOOL_FRAMES = [     # USED TO Z-SHIFT THE ROTATION COMPONENT OF THE ERROR VECTOR
    array([ # middle of gripper frame
        [1., 0., 0., 0.],
        [0., 1., 0., 0.],
        [0., 0., 1., 0.],
        [0., 0., 0., 1.]
    ]),
    array([ # end of gripper frame
        [1., 0., 0., 0.    ],
        [0., 1., 0., 0.    ],
        [0., 0., 1., 0.14844], #0.3048 was too large of a value
        [0., 0., 0., 1.    ]
    ]),
    array([ # wrist gripper frame
        [1., 0., 0.,  0.     ],
        [0., 1., 0.,  0.     ],
        [0., 0., 1., -0.14844],
        [0., 0., 0.,  1.     ]
    ]),
    array([ # solenoid frame
        [1.,  0., 0., 0.    ],
        [0.,  0., 1., 0.2159],
        [0., -1., 0., 0.    ],
        [0.,  0., 0., 1.    ]
    ]),
    array([ # hexkey frame
        [1., 0.,  0.,  0.    ],
        [0., 0., -1., -0.2159],
        [0., 1.,  0.,  0.    ],
        [0., 0.,  0.,  1.    ]
    ])
]



if __name__ == "__main__":
    print("The DH parameters for the rover arm are:")
    [print(joint) for joint in dh_params]

    arm_to_plot = DHRobot(dh_params, name="For demonstration purposes. Does it match the URDF file when all joints at 0?")
    arm_to_plot.plot(INITIAL_Q, block=True)
