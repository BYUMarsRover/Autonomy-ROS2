#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from rover_msgs.msg import IWCMotors, Elevator
from mobility.controllers.teleop_controllers import TankController, ArcadeController

# Ammon Test Changes
import joysticks.publish_on_update as pubup
import joysticks.input_tools as iptl

# TODO: put these inside a yaml
# Button and axis mappings
A, B, X, Y, LB, RB = 0, 1, 2, 3, 4, 5
BACK, START, POWER = 6, 7, 8 # 6: Disable drive, 7: Enable drive
BUTTON_STICK_LEFT, BUTTON_STICK_RIGHT = 9, 10

LEFT_STICK_HORIZONTAL, LEFT_STICK_VERTICAL = 0, 1
LT, RIGHT_STICK_HORIZONTAL, RIGHT_STICK_VERTICAL, RT = 2, 3, 4, 5
DPAD_HORIZONTAL, DPAD_VERTICAL = 6, 7

ELEVATOR_DIR_UP, ELEVATOR_DIR_DOWN = 1, 0

MIN_BUFFER, MAX_BUFFER = 0.02, 0.98

SPEED_CONSTANTS = [0.05, 0.1, 0.3, 0.5, 0.7, 1]
NUM_OF_SPEED_CONSTANTS = len(SPEED_CONSTANTS)
ELEVATOR_SPEED_CONSTANTS = [0.3, 0.5, 0.7, 1]
NUM_OF_ELEVATOR_SPEED_CONSTANTS = len(ELEVATOR_SPEED_CONSTANTS)


class XBOX(Node):

    def __init__(self):
        super().__init__('xbox_drive')
        self.get_logger().info('Running XBOX Drive')

        # Subscribers
        self.sub_joy_sub = self.create_subscription(
            Joy,
            '/joy_drive_input',
            self.joy_callback,
            10
        )

        # Publishers
        self.publisher_group = pubup.UpdatePublisherGroup()
        self.joy_drive_enabled_pub = self.publisher_group.add_publisher(
            pubup.UpdatePublisher(
                self.create_publisher(Bool, "/mobility/joy_drive_enabled", 10),
                "std_msgs.msg.Bool"
                )
            )
        self.elevator_pub = self.publisher_group.add_publisher(pubup.ElevatorPublisher(self.create_publisher(Elevator, "/elevator", 10)))

        self.teleop_drive_cmds_pub = self.create_publisher(
            IWCMotors,
            '/mobility/teleop_drive_cmds',
            10
        )

        self.drive_enabled = False
        self.drive_speed_multiplier_idx = 0
        self.drive_speed_multiplier = SPEED_CONSTANTS[self.drive_speed_multiplier_idx]
        self.last_left_bumper = False
        self.last_right_bumper = False

        self.last_left_dpad = False
        self.last_right_dpad = False
        self.elevator_speed_multiplier_idx = 0
        self.elevator_speed_multiplier = ELEVATOR_SPEED_CONSTANTS[self.elevator_speed_multiplier_idx]

        self.drivetrain_mode = 'tank'

    def joy_callback(self, msg: Joy):
        IWC_cmd_msg = IWCMotors()

        self._check_drive_enabled(msg)

        if self.drive_enabled:
            self._check_desired_drive_speed(msg)
            self._check_drive_mode(msg)
            
            if self.drivetrain_mode == 'tank':
                IWC_cmd_msg = self.tank_drive(msg)
            elif self.drivetrain_mode == 'arcade':
                IWC_cmd_msg = self.arcade_drive(msg)
            else:
                self.get_logger().error('Invalid Drivetrain mode')
                return

            self.teleop_drive_cmds_pub.publish(IWC_cmd_msg)

            # Call to update elevator commands
            self.elevator_commands(msg)

        # Post all changes to the publishers
        self.publisher_group.publish()


    def elevator_commands(self, msg: Joy):
        '''Updates the elevator publisher with new commands'''

        elevator_input = msg.axes[DPAD_VERTICAL]
        elevator_speed_input = msg.axes[DPAD_HORIZONTAL]

        self.elevator_pub.set_speed(int(self.elevator_speed_multiplier * 255))

        if elevator_input == 1.0:
            self.elevator_pub.set_dir(ELEVATOR_DIR_UP)
        elif elevator_input == -1.0:
            self.elevator_pub.set_dir(ELEVATOR_DIR_DOWN)
        else: 
            self.elevator_pub.set_speed(0)
        
        if elevator_speed_input == 1:
            left_dpad = True
            right_dpad = False 
        elif elevator_speed_input == -1:
            right_dpad = True 
            left_dpad = False 
        else:
            left_dpad = False
            right_dpad = False      

        if left_dpad and not self.last_left_dpad:
            if self.elevator_speed_multiplier_idx > 0:
                self.elevator_speed_multiplier_idx -= 1
                self.elevator_speed_multiplier = ELEVATOR_SPEED_CONSTANTS[self.elevator_speed_multiplier_idx]
            self.last_left_dpad = True
        
        elif right_dpad and not self.last_right_dpad:
            if self.elevator_speed_multiplier_idx < NUM_OF_ELEVATOR_SPEED_CONSTANTS - 1:
                self.elevator_speed_multiplier_idx += 1
                self.elevator_speed_multiplier = ELEVATOR_SPEED_CONSTANTS[self.elevator_speed_multiplier_idx]
            self.last_right_dpad = True
        
        if not left_dpad:
            self.last_left_dpad = False
        if not right_dpad:
            self.last_right_dpad = False


    def tank_drive(self, msg: Joy):
        left_vertical_axis = msg.axes[LEFT_STICK_VERTICAL]
        right_vertical_axis = msg.axes[RIGHT_STICK_VERTICAL]

        left_wheels = self._apply_buffer(left_vertical_axis)
        right_wheels = self._apply_buffer(right_vertical_axis)

        left_wheels *= self.drive_speed_multiplier
        right_wheels *= self.drive_speed_multiplier

        TC = TankController()
        return TC.control(left_wheels, right_wheels)

    def arcade_drive(self, msg: Joy):
        left_vertical_axis = msg.axes[LEFT_STICK_VERTICAL]
        right_horizontal_axis = msg.axes[RIGHT_STICK_HORIZONTAL]

        y = self._apply_buffer(left_vertical_axis)
        x = self._apply_buffer(right_horizontal_axis)

        x *= self.drive_speed_multiplier
        y *= self.drive_speed_multiplier

        AC = ArcadeController()
        return AC.control(x, y)

    def _apply_buffer(self, value):
        if abs(value) < MIN_BUFFER:
            return 0
        elif abs(value) > MAX_BUFFER:
            return -1 if value < 0 else 1
        return value

    def _check_drive_enabled(self, msg: Joy):
        start_button = msg.buttons[START]
        back_button = msg.buttons[BACK]

        if back_button:
            if (self.drive_enabled):
                self.drive_enabled = False
                self.joy_drive_enabled_pub.set(self.drive_enabled)
                self.get_logger().info('Drive disabled')
        elif start_button:
            if (not self.drive_enabled):
                self.drive_enabled = True
                self.joy_drive_enabled_pub.set(self.drive_enabled)
                self.get_logger().info('Drive enabled')

    def _check_desired_drive_speed(self, msg: Joy):
        right_bumper = msg.buttons[RB]
        left_bumper = msg.buttons[LB]

        if left_bumper and not self.last_left_bumper:
            if self.drive_speed_multiplier_idx > 0:
                self.drive_speed_multiplier_idx -= 1
                self.drive_speed_multiplier = SPEED_CONSTANTS[self.drive_speed_multiplier_idx]
            self.last_left_bumper = True
        
        elif right_bumper and not self.last_right_bumper:
            if self.drive_speed_multiplier_idx < NUM_OF_SPEED_CONSTANTS - 1:
                self.drive_speed_multiplier_idx += 1
                self.drive_speed_multiplier = SPEED_CONSTANTS[self.drive_speed_multiplier_idx]
            self.last_right_bumper = True
        
        if not left_bumper:
            self.last_left_bumper = False
        if not right_bumper:
            self.last_right_bumper = False

    def _check_drive_mode(self, msg: Joy):
        x_button = msg.buttons[X]
        y_button = msg.buttons[Y]

        if x_button and not self.drivetrain_mode == 'tank':
            self.drivetrain_mode = 'tank'
            self.get_logger().info('Drive mode is now tank')
        elif y_button and not self.drivetrain_mode == 'arcade':
            self.drivetrain_mode = 'arcade'
            self.get_logger().info('Drive mode is now arcade')

def main(args=None):
    rclpy.init(args=args)
    xbox = XBOX()
    rclpy.spin(xbox)
    xbox.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()