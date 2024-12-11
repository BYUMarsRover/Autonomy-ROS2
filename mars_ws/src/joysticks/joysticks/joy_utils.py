from rover_msgs.msg import Xbox

##############################
# MAPPING OF JOY MSG TO XBOX #
##############################
# See http://wiki.ros.org/joy
# Only for xbox 360 controllers: xbox one has different order

A_BTN = "A"
B_BTN = "B"
X_BTN = "X"
Y_BTN = "Y"
L_BUMPER = "L_BUMPER"  # left bumper, not trigger
R_BUMPER = "R_BUMPER"
BACK_BTN = "BACK"
START_BTN = "START"
HOME_BTN = "XBOX"  # xbox button
L_STICK_PRESS = "L_STICK_PRESS"  # pressing the left stick in
R_STICK_PRESS = "R_STICK_PRESS"
L_STICK_LR = "L_STICK_LR"  # Moving left stick left and right
L_STICK_UD = "L_STICK_UD"  # Up and down
L_TRIGGER = "L_TRIGGER"  # Left trigger, not the bumper
R_STICK_LR = "R_STICK_LR"
R_STICK_UD = "R_STICK_UD"
R_TRIGGER = "R_TRIGGER"
D_PAD_LR = "D_PAD_LR"  # Pressing the cross left or right
D_PAD_UD = "D_PAD_UD"

# Buttons (1 if pressed, 0 otherwise)
BUTTON_NAMES_IN_ORDER = [
    A_BTN,
    B_BTN,
    X_BTN,
    Y_BTN,
    L_BUMPER,
    R_BUMPER,
    BACK_BTN,
    START_BTN,
    HOME_BTN,
    L_STICK_PRESS,
    R_STICK_PRESS,
]
IDX_TO_BUTTON = {idx: name for idx, name in zip(range(11), BUTTON_NAMES_IN_ORDER)}
BUTTON_TO_IDX = {name: idx for idx, name in zip(range(11), BUTTON_NAMES_IN_ORDER)}

# Axes (each axis goes -1 to 1; 1 for left or up, -1 right or down; for the triggers, 1 unpressed -1 pressed)
AXES_NAMES_IN_ORDER = [
    L_STICK_LR,
    L_STICK_UD,
    L_TRIGGER,
    R_STICK_LR,
    R_STICK_UD,
    R_TRIGGER,
    D_PAD_LR,
    D_PAD_UD,
]
IDX_TO_AXIS = {idx: name for idx, name in zip(range(8), AXES_NAMES_IN_ORDER)}
AXIS_TO_IDX = {name: idx for idx, name in zip(range(8), AXES_NAMES_IN_ORDER)}


# Dissects an xbox message into useful dictionaries
def joy_msg_to_dict(xbox_msg):
    button_dict = {
        name: value for name, value in zip(BUTTON_NAMES_IN_ORDER, xbox_msg.buttons)
    }
    axes_dict = {name: value for name, value in zip(AXES_NAMES_IN_ORDER, xbox_msg.axes)}
    return button_dict, axes_dict


def joy_msg_to_xbox_msg(joy_msg, deadzone=0.0):
    button_dict, axes_dict = joy_msg_to_dict(joy_msg)
    xbox_msg = Xbox()

    # A little bit of python magic to avoid repetitive code
    for name, value in list(button_dict.items()) + list(axes_dict.items()):
        # Map triggers to better range
        if name in {R_TRIGGER, L_TRIGGER}:
            value = 0.5 * (1 - value)

        # Check deadzone
        if isinstance(value, float) and abs(value) < deadzone:
            value = 0.0

        # Sets xbox_msg.A to button_dict["A"], etc.
        xbox_msg.__setattr__(name, value)

    return xbox_msg