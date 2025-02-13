#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
# from rover_msgs.msg import (
#     Elevator,
#     ScienceLinearActuatorDirection,
#     ScienceCacheDoor,
#     ScienceAugerOn,
#     ScienceSwitchTool,
#     ScienceSaveSecondaryCache,
#     ScienceSecondaryCachePosition
# )
from rover_msgs.msg import (
    Elevator,
    ScienceActuatorControl
)

# Left joy vertical -> elevator X
# Right joy vertical -> linear actuator X
# Bumpers -> switch tool X
# Right trigger -> drill forward X
# Left trigger -> drill reverse X
# A button -> toggle linear actuator (trap door) X
# Back button -> toggle secondary cache out or in
# Xbox button -> move sample to secondary cache

A = 0  # Toggle trap door linear actuator
B = 1  # Emergency stop secondary cache automation
X = 2
Y = 3
LB = 4  # Switch actuator to previous
RB = 5  # Switch actuator to next
BACK = 6  # Toggle secondary cache out or in
START = 7
POWER = 8  # (Xbox) Move sample to secondary cache
BUTTON_STICK_LEFT = 9
BUTTON_STICK_RIGHT = 10

LEFT_STICK_HORIZONTAL = 0
LEFT_STICK_VERTICAL = 1  # Move elevator up/down
LT = 2  # Drill reverse
RIGHT_STICK_HORIZONTAL = 3
RIGHT_STICK_VERTICAL = 4  # Move linear actuator up/down
RT = 5  # Drill forward
DPAD_HORIZONTAL = 6 # Move secondary cache linear actuator in and out
DPAD_VERTICAL = 7

CACHE_DOOR_OPEN = True
CACHE_DOOR_CLOSED = False

FULL_STEAM_FORWARD = 127
FULL_STEAM_BACKWARD = -128

D_PAD_PREF = -1 # Change this to switch D-Pad Axis convention


class XBOX(Node):

    DEAD_ZONE = 0.05 # Default Deadzone

    def __init__(self):
        super().__init__("xbox_science")

        self.get_logger().info('running joystick!')

        # Subscribers
        self.sub_joy = self.create_subscription(
            Joy, "/joy_science_input", self.joy_callback, 10
        )

        # Publishers
        self.pub_elevator = self.create_publisher(Elevator, "/elevator", 10)

        self.pub_drill = self.create_publisher(ScienceActuatorControl, '/science_serial_drill', 10)
        self.pub_probe = self.create_publisher(ScienceActuatorControl, "/science_serial_probe", 10)
        self.pub_auger = self.create_publisher(ScienceActuatorControl, "/science_serial_auger", 10)
        self.pub_primary_cache_door = self.create_publisher(ScienceActuatorControl, "/science_serial_primary_cache_door", 10)
        self.pub_secondary_cache = self.create_publisher(ScienceActuatorControl, "/science_serial_secondary_cache", 10)
        
        # Define Actuator Control Objects
        self.drill_control = self.ActuatorController_Axis(self.pub_drill)
        self.probe_control = self.ActuatorController_Axis(self.pub_probe)
        self.auger_control = self.ActuatorController_Axis(self.pub_auger)
        self.primary_cache_door_control = self.ActuatorController_UnsignedInt(self.pub_primary_cache_door)
        self.secondary_cache_control = self.ActuatorController_Axis(self.pub_secondary_cache)
        self.actuator_controls = [self.drill_control, self.probe_control, self.auger_control, self.primary_cache_door_control, self.primary_cache_door_control, self.secondary_cache_control]

        # Initialize state variables
        self.elev_direction = 0
        self.prev_joy_state = None
        self.using_probe = False

        # Set up input objects
        self.drill_forward_binding = self.InputBinding_Throttle(RT, dead_zone=0.01)
        self.drill_backward_binding = self.InputBinding_Throttle(LT, dead_zone=0.01)
        self.tool_binding = self.InputBinding_Axis(RIGHT_STICK_VERTICAL, invert=True)
        self.primary_cache_door_binding = self.InputBinding_Button(A, FULL_STEAM_FORWARD, FULL_STEAM_BACKWARD)
        self.secondary_cache_binding = self.InputBinding_Axis(DPAD_HORIZONTAL, invert=True)

    class ActuatorController_UnsignedInt():
        '''Tracks the control for an actuator and publishes the change'''

        def __init__(self, pub):
            self.pub = pub
            self.control = 0
            self.is_updated = False

        def set(self, new_control):
            self.is_updated = new_control != self.control # Set this flag if control has updated
            self.control = new_control

        def check_publish(self):
            if (self.is_updated):
                self.pub.publish(ScienceActuatorControl(control=self.control))
                self.is_updated = False

    class ActuatorController_Axis(ActuatorController_UnsignedInt):

        # Overrides the base set and converts the [-1, 1] float range to [-127, 127] int range
        def set(self, new_control):
            super().set(int(new_control * 127))

    class InputBinding_Button:
        '''When the button is held, the control will be in the on state, off otherwise'''

        def __init__(self, button_index, control_on, control_off=0):
            self.button_index = button_index
            self.control_on = control_on
            self.control_off = control_off
            self.state = False

        def button_logic(self, msg: Joy):
            return msg.buttons[self.button_index]

        def update(self, msg: Joy):
            self.state = self.button_logic(msg)
            return (self.control_on if self.state else self.control_off)

    class InputBinding_Toggle(InputBinding_Button):
        '''When the button is pressed, the control will switch between the provided off and on states'''

        def button_logic(self, msg: Joy, prev_msg: Joy):
            # When the button is pressed, toggle between on and off
            if msg.buttons[self.button_index] != prev_msg.buttons[self.button_index] and msg.buttons[self.button_index]:
                return not self.state
            else:
                return self.state
            
        def update(self, msg: Joy, prev_msg: Joy):
            if prev_msg is not None:
                self.state = self.button_logic(msg, prev_msg)
                return (self.control_on if self.state else self.control_off)

    class InputBinding_Axis:
        '''When the axis is outside of the deadzone, its control is converted linearly and sent to actuator'''

        def __init__(self, axis_index, invert=False, dead_zone=0.05):
            self.axis_index = axis_index
            self.invert = invert
            self.dead_zone = dead_zone

        def update(self, msg: Joy):
            if abs(msg.axes[self.axis_index]) > self.dead_zone:
                control = (-1 if self.invert else 1) * msg.axes[self.axis_index]
            else:
                control = 0
            return control

    class InputBinding_Throttle:
        '''When the axis is outside of the deadzone, its control is converted linearly and sent to actuator'''

        def __init__(self, axis_index, invert=False, dead_zone=0.05):
            self.axis_index = axis_index
            self.invert = invert
            self.dead_zone = dead_zone

        def update(self, msg: Joy):
            if msg.axes[self.axis_index] < -self.dead_zone:
                control = (-1 if self.invert else 1) * -msg.axes[self.axis_index]
            else:
                control = 0
            return control

    def handle_elevator(self, msg: Joy):
        # Elevator Control: LEFT JOYSTICK VERTICAL
        DEAD_ZONE = 0.05
        if msg.axes[LEFT_STICK_VERTICAL] > DEAD_ZONE:
            elev_direction = 1
        elif msg.axes[LEFT_STICK_VERTICAL] < -DEAD_ZONE:
            elev_direction = -1
        else:
            elev_direction = 0

        if elev_direction != self.elev_direction:
            self.elev_direction = elev_direction
            real_elev_direction = 1 if elev_direction > 0 else 0
            elev_speed = int(255 * 0.9 if elev_direction != 0 else 0)
            elev_msg = Elevator()
            elev_msg.elevator_direction = real_elev_direction
            elev_msg.elevator_speed = elev_speed
            self.pub_elevator.publish(elev_msg)


    def joy_callback(self, msg: Joy):

        # update Input Objects
        input_throttle_drill_forward =      self.drill_forward_binding.update(msg)
        input_throttle_drill_backward =     self.drill_backward_binding.update(msg)
        input_tool =                        self.tool_binding.update(msg)
        input_button_primary_cache_door =   self.primary_cache_door_binding.update(msg)
        input_axis_secondary_cache =        self.secondary_cache_binding.update(msg)

        # Drill Control - Add the control signals sent by each throttle
        self.drill_control.set(input_throttle_drill_forward - input_throttle_drill_backward)

        # Handle Primary and Secondary Caches
        self.primary_cache_door_control.set(input_button_primary_cache_door)
        self.secondary_cache_control.set(input_axis_secondary_cache)
        
        # Switch Tool: RIGHT AND LEFT BUMPERS - Change it so that based on switch_tool_direction, publishes to a different topic
        if (msg.buttons[LB] and not self.prev_joy_state.buttons[LB]) or (msg.buttons[RB] and not self.prev_joy_state.buttons[RB]):

            # Turn of an actuator before we switch contexts
            if (self.using_probe):
                self.probe_control.set(0)
            else:
                self.auger_control.set(0)

            # Switch Tool Context
            self.using_probe = not self.using_probe

        # Only send input commands the selected tool
        if (self.using_probe):
            self.probe_control.set(input_tool)
        else:
            self.auger_control.set(input_tool)

        # Publish any changes to the actuators if anything has been updated
        for actuator in self.actuator_controls:
            actuator.check_publish()

        # Publish elevator commands
        self.handle_elevator(msg)

        # Record previous state so that we can handle toggle buttons more easily
        self.prev_joy_state = msg


def main(args=None):
    rclpy.init(args=args)
    xbox = XBOX()
    rclpy.spin(xbox)
    xbox.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
