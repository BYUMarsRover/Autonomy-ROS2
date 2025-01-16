#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from rover_msgs.msg import DriveEnable, IWCMotors
from drivetrain_controllers import TankController, VelocityHeadingController, ArcadeController

A = 0
B = 1
X = 2
Y = 3
LB = 4
RB = 5
BACK = 6  # Disable drive
START = 7  # Enable drive
POWER = 8
BUTTON_STICK_LEFT = 9
BUTTON_STICK_RIGHT = 10

LEFT_STICK_HORIZONTAL = 0
LEFT_STICK_VERTICAL = 1  # Left drive
LT = 2
RIGHT_STICK_HORIZONTAL = 3
RIGHT_STICK_VERTICAL = 4  # Right drive
RT = 5
DPAD_HORIZONTAL = 6
DPAD_VERTICAL = 7

MIN_BUFFER = .02
MAX_BUFFER = .98

SPEED_CONSTANTS = [0.05, 0.1, 0.3, 0.5, 0.7, 1]
NUM_OF_SPEED_CONSTANTS = len(SPEED_CONSTANTS)

class XBOX(Node):

    def __init__(self):
        super().__init__('xbox_drive')

        # Subscribers
        self.sub_joy = self.create_subscription(Joy, '/joy_drive_input', self.joy_callback, 10)

        # Publishers
        self.IWC_control = self.create_publisher(IWC_motors, '/IWC_motorControl', 10)
        self.pub_drive_enable = self.create_publisher(DriveEnable, '/drive_enable', 10)

        self.drive_enabled = False
        self.drive_speed_multiplier_idx = 0
        self.drive_speed_multiplier = SPEED_CONSTANTS[self.drive_speed_multiplier_idx]
        self.last_left_bumper = False
        self.last_right_bumper = False

        self.drivetrain_mode = 'tank'  # False is for single joystick drive

    def joy_callback(self, msg: Joy):
        # Wheel Drive
        IWC_cmd_msg = IWC_motors()

        # check if drive is active
        self._check_drive_enabled(msg)

        # check desired drive speed
        self._check_desired_drive_speed(msg)

        # check drive mode
        self._check_drive_mode(msg)

        # calculate IWC with desired drivetrain mode
        if self.drivetrain_mode == 'tank':
            # do tank drive
            IWC_cmd_msg = self.tank_drive(msg)
        elif self.drivetrain_mode == 'arcade':
            # do arcade drive
            IWC_cmd_msg = self.arcade_drive(msg)
        else:
            raise Exception('Invalid Drivetrain mode')

        # publish IWC msg
        if self.drive_enabled:
            self.get_logger().info('Publishing IWC_cmds...')
            self.IWC_control.publish(IWC_cmd_msg)

    def tank_drive(self, msg: Joy):
        left_vertical_axis = msg.axes[LEFT_STICK_VERTICAL]
        right_vertical_axis = msg.axes[RIGHT_STICK_VERTICAL]

        # Set Buffer Zones
        if abs(left_vertical_axis) < MIN_BUFFER:
            left_wheels = 0
        elif abs(left_vertical_axis) > MAX_BUFFER:
            left_wheels = -1 if left_vertical_axis < 0 else 1
        else:
            left_wheels = left_vertical_axis
        
        if abs(right_vertical_axis) < MIN_BUFFER:
            right_wheels = 0
        elif abs(right_vertical_axis) > MAX_BUFFER:
            right_wheels = -1 if right_vertical_axis < 0 else 1
        else:
            right_wheels = right_vertical_axis

        # apply drive speed multiplier
        left_wheels *= self.drive_speed_multiplier
        right_wheels *= self.drive_speed_multiplier

        TC = TankController()
        IWC_cmd_msg = TC.control(left_wheels, right_wheels)

        return IWC_cmd_msg

    def arcade_drive(self, msg: Joy):
        left_vertical_axis = msg.axes[LEFT_STICK_VERTICAL]
        right_horizontal_axis = msg.axes[RIGHT_STICK_HORIZONTAL]

        if abs(left_vertical_axis) < MIN_BUFFER:
            left_vertical_axis = 0
        elif abs(left_vertical_axis) > MAX_BUFFER:
            left_vertical_axis = -1 if left_vertical_axis < 0 else 1

        if abs(right_horizontal_axis) < MIN_BUFFER:
            right_horizontal_axis = 0
        elif abs(right_horizontal_axis) > MAX_BUFFER:
            right_horizontal_axis = -1 if right_horizontal_axis < 0 else 1

        x, y = right_horizontal_axis * self.drive_speed_multiplier, left_vertical_axis * self.drive_speed_multiplier
        AC = ArcadeController()
        IWC_cmd_msg = AC.control(x, y)

        return IWC_cmd_msg

    def _check_drive_enabled(self, msg: Joy):
        start_button = msg.buttons[START]
        back_button = msg.buttons[BACK]

        if back_button:
            self.get_logger().info('Drive disabled')
            self.drive_enabled = False
        elif start_button:
            self.get_logger().info('Drive enabled')
            self.drive_enabled = True

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

        if x_button:
            self.drivetrain_mode = 'tank'
            self.get_logger().info('Drive mode is now tank')
        elif y_button:
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
