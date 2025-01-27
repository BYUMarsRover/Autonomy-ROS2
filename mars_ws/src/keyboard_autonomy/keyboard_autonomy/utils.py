"""
Utility/helper functions and classes for arm control package
"""

from enum import Enum, auto
from numpy import pi

######################
# OTHER MISC HELPERS #
######################

DEG_TO_RAD = pi / 180
# Bring angle between -pi and pi
def normalize_angle(angle, radians=True):
    angle = angle if radians else angle * DEG_TO_RAD
    normed = (angle + pi) % (2 * pi) - pi
    return normed if radians else normed / DEG_TO_RAD


# Enum for Mode Type
class ArmControlMode(Enum):
    JOINT_CONTROL = auto()  # Joint by joint
    IK_BASE_FRAME = auto()  # IK in the base frame


# All the different things the arm control controls for JBJ
class Controllable(Enum):
    NODE_1 = 1
    NODE_2 = 2
    NODE_3 = 3
    NODE_4 = 4
    NODE_5 = 5
    GRIPPER = auto()
    ELEVATOR = auto()
    NONE_SELECTED = auto()


def check_lim(new_q, qlim_bot, qlim_top, v=None):
    """
    Checks the joint limits for an individual joint.
    --new_q: the angle in radians that the joint WILL BE at after motion (should be -pi to pi)
    --qlim_bot: the bottom joint limit in radians
    --qlim_top: the top joint limit in radians
    --v: radian velocity (optional). If provided, this function will allow velocities
        that move the joint towards allowed ranges between joint limits
    --returns: boolean. True: valid joint angle. False: invalid, caller should zero out the velocity
        to stay within joint limits
    """
    if v is None:
        return (new_q <= qlim_top) and (new_q >= qlim_bot)
    return ((new_q <= qlim_top) or v <= 0) and ((new_q >= qlim_bot) or v >= 0)


def enforce_joint_limits(new_qs, vs, qlim_bots, qlim_tops):
    """
    Returns v with invalid velocities zeroed out.
    --new_qs: list of angles in radians that the joints WILL BE at after motion (should be -pi to pi)
    --vs: list of radian velocities for each joint. This function will allow velocities that move the
        joint towards allowed ranges between joint limits
    --qlim_bots: the bottom joint limits in radians
    --qlim_tops: the top joint limits in radians
    --returns: tuple(list, list). First list is booleans of whether each velocity was valid--False
        means that velocity was zeroed out. Second list is new velocities with invalid ones zeroed
    """
    assert (
        len(new_qs) == len(vs) == len(qlim_bots) == len(qlim_tops)
    ), "Cannot enforce with different length elements"
    valid = [
        check_lim(q, bot, top, v=v)
        for q, v, bot, top in zip(new_qs, vs, qlim_bots, qlim_tops)
    ]
    new_vs = [v if val else 0.0 for v, val in zip(vs, valid)]
    return (valid, new_vs)