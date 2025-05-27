#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Empty
from rover_msgs.msg import (
    Elevator,
    ScienceActuatorControl
)

import joysticks.publish_on_update as pubup
import joysticks.input_tools as iptl

# Left joy vertical -> elevator X
# Right joy vertical -> linear actuator X
# Bumpers -> switch tool X
# Right trigger -> drill forward X
# Left trigger -> drill reverse X
# A button -> toggle linear actuator (trap door) X
# Back button -> toggle secondary cache out or in
# Xbox button -> move sample to secondary cache

A = 0  # Toggle trap door linear actuator
B = 1
X = 2
Y = 3
LB = 4  # Switch actuator to previous
RB = 5  # Switch actuator to next
BACK = 6  
START = 7
POWER = 8
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

FULL_STEAM_FORWARD = 127
FULL_STEAM_BACKWARD = -128


class XBOX(Node):

    def __init__(self):
        super().__init__("xbox_science")
        self.get_logger().info('Running XBOX Science')

        # Subscribers
        self.sub_joy = self.create_subscription(
            Joy, "/joy_science_input", self.joy_callback, 10
        )
        self.sub_auger = self.create_subscription(
            Empty, "/science/auger_position", self.auger_control_callback, 10
        )

        # Publishers
        self.setup_update_publishers()
        self.pub_using_probe = self.create_publisher(
            Bool, "/science/using_probe", 10
        )
        self.pub_reset_science_module = self.create_publisher(
            Empty, '/science/serial/reset', 10
        )


        # Initialize state variables
        self.prev_joy_state = None
        self.using_probe = False

        # Set up input objects
        # These handle the logic for input interactions
        self.drill_forward_throttle = iptl.InputThrottle(RT, dead_zone=0.01)
        self.drill_backward_throttle = iptl.InputThrottle(LT, dead_zone=0.01)
        self.tool_axis = iptl.InputAxis(RIGHT_STICK_VERTICAL, invert=True)
        self.elevator_axis = iptl.InputAxis(LEFT_STICK_VERTICAL)
        self.primary_cache_door_button = iptl.InputButton(A, FULL_STEAM_FORWARD, FULL_STEAM_BACKWARD)
        self.secondary_cache_axis = iptl.InputAxis(DPAD_HORIZONTAL, invert=True)
        self.reset_flag = iptl.ButtonFlag(iptl.InputHold(BACK, 1000))
        self.override_button = iptl.InputButton(POWER, True, False)

    def auger_control_callback(self, msg: Empty):
        self.pub_using_probe.publish(Bool(data=self.using_probe))

    def joy_callback(self, msg: Joy):
        '''Logic goes here'''

        # Update Input Objects
        # This updates all the button, axis, throttle logic etc.
        input_throttle_drill_forward =      self.drill_forward_throttle.update(msg)
        input_throttle_drill_backward =     self.drill_backward_throttle.update(msg)
        input_axis_tool =                   self.tool_axis.update(msg)
        input_button_primary_cache_door =   self.primary_cache_door_button.update(msg)
        input_axis_secondary_cache =        self.secondary_cache_axis.update(msg)
        input_override =                    self.override_button.update(msg)
        input_elevator =                    self.elevator_axis.update(msg)
        input_reset_flag =                  self.reset_flag.update(msg)

        # Handle Reset - Trigger the reset topic
        if input_reset_flag:
            self.get_logger().info("Resetting science module...")
            self.reset_flag.acknowledge()
            self.pub_reset_science_module.publish(Empty())

        # Handle Override - This sets the override bit for science module communications (see science_serial.py)
        self.override_control.set(input_override)

        # Handle Drill - Add the control signals sent by each throttle
        self.drill_control.set(input_throttle_drill_forward - input_throttle_drill_backward)

        # Handle Primary and Secondary Caches
        self.primary_cache_door_control.set(input_button_primary_cache_door)
        self.secondary_cache_control.set(input_axis_secondary_cache)
        
        # Handle Switching Tools
        if (self.prev_joy_state is not None):
            if (msg.buttons[LB] and not self.prev_joy_state.buttons[LB]) or (msg.buttons[RB] and not self.prev_joy_state.buttons[RB]):

                # Turn of an actuator before we switch contexts
                if (self.using_probe):
                    self.probe_control.set(0)
                else:
                    self.auger_control.set(0)

                # Switch Tool Context
                self.using_probe = not self.using_probe
                self.auger_control_callback(None)

        # Handle Probe and Auger
        if (self.using_probe):
            self.probe_control.set(input_axis_tool)
        else:
            self.auger_control.set(input_axis_tool)

        # Handle Elevator
        self.elevator_control.set_dir(1 if input_elevator > 0 else 0)
        self.elevator_control.set_speed(int(255 * 0.9 if input_elevator != 0 else 0))

        # Publish all changes
        self.update_publisher.publish()

        # Record previous state so that we can handle edge detection
        self.prev_joy_state = msg

    def setup_update_publishers(self):
        '''Setup the update publishing groups'''

        # Set up the publishing group
        self.update_publisher = pubup.UpdatePublisherGroup()

        # Elevator Publisher
        self.elevator_control = self.update_publisher.add_publisher(
            pubup.ElevatorPublisher(
                self.create_publisher(Elevator, "/elevator", 10)
                )
            )
        
        # Drill Publisher
        self.drill_control = self.update_publisher.add_publisher(
            pubup.ScienceActuatorPublisher(
                self.create_publisher(ScienceActuatorControl, '/science/serial/drill', 10),
                format_func=pubup.ScienceActuatorPublisher.convert_axis_to_unsigned_integer
                )
            )
        
        # Probe Publisher
        self.probe_control = self.update_publisher.add_publisher(
            pubup.ScienceActuatorPublisher(
                self.create_publisher(ScienceActuatorControl, "/science/serial/probe", 10),
                format_func=pubup.ScienceActuatorPublisher.convert_axis_to_unsigned_integer
                )
            )
        
        # Auger Publisher
        self.auger_control = self.update_publisher.add_publisher(
            pubup.ScienceActuatorPublisher(
                self.create_publisher(ScienceActuatorControl, "/science/serial/auger", 10),
                format_func=pubup.ScienceActuatorPublisher.convert_axis_to_unsigned_integer
                )
            )
        
        # Primary Cache Publisher
        self.primary_cache_door_control = self.update_publisher.add_publisher(
            pubup.ScienceActuatorPublisher(
                self.create_publisher(ScienceActuatorControl, "/science/serial/primary_cache_door", 10)
                )
            )
        
        # Secondary Cache Publisher
        self.secondary_cache_control = self.update_publisher.add_publisher(
            pubup.ScienceActuatorPublisher(
                self.create_publisher(ScienceActuatorControl, "/science/serial/secondary_cache", 10),
                format_func=pubup.ScienceActuatorPublisher.convert_axis_to_unsigned_integer
                )
            )
        
        # Override Publisher
        self.override_control = self.update_publisher.add_publisher(
            pubup.UpdatePublisher(
                self.create_publisher(Bool, "/science/serial/override", 10),
                "std_msgs.msg.Bool"
                )
            )
        
def main(args=None):
    rclpy.init(args=args)
    xbox = XBOX()
    rclpy.spin(xbox)
    xbox.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
