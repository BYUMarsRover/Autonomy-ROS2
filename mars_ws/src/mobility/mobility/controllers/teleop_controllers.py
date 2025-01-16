#!/usr/bin/env python3

from rover_msgs.msg import IWCMotors
import numpy as np

class TeleopController():
    def __init__(self):
        pass

    def control(self) -> IWCMotors:
        pass

    def _check_speed(self, val):
        '''
        Ensures that the val is a float and is between 0-255 
        '''
        return int(abs(val) * 255.)

    def _check_dir(self, val):
        return val >= 0

class TankController(TeleopController):
    """
    Tank control is differential drive control. All the left wheels drive at the same speed, all
    the right wheels drive at the same speed. Speeding up 1 side or the other turns the rover.
    """
    def __init__(self):
        TeleopController.__init__(self)

    def control(self, left_wheels: float, right_wheels: float) -> IWCMotors:
        """
        Computes IWC commands given a left_wheels and right_wheels input
        left_wheels: float: a percentage of total speed for the left wheels (0-1)
        right_wheels: float: a percentage of total speed for the right wheels (0-1)
        """
        left_dir = self._check_dir(left_wheels)
        right_dir = self._check_dir(right_wheels)

        left_speed = self._check_speed(left_wheels)
        right_speed = self._check_speed(right_wheels)

        # Create IWC Message
        IWC_cmd_msg = IWCMotors()
        IWC_cmd_msg.right_front_speed = abs(right_speed)
        IWC_cmd_msg.right_front_dir = right_dir

        IWC_cmd_msg.right_middle_speed = abs(right_speed)
        IWC_cmd_msg.right_middle_dir = right_dir

        IWC_cmd_msg.right_rear_speed = abs(right_speed)
        IWC_cmd_msg.right_rear_dir = right_dir

        IWC_cmd_msg.left_front_speed = abs(left_speed)
        IWC_cmd_msg.left_front_dir = left_dir

        IWC_cmd_msg.left_middle_speed = abs(left_speed)
        IWC_cmd_msg.left_middle_dir = left_dir

        IWC_cmd_msg.left_rear_speed = abs(left_speed)
        IWC_cmd_msg.left_rear_dir = left_dir

        print(IWC_cmd_msg)

        # Return IWC cmds
        return IWC_cmd_msg

class ArcadeController(TeleopController):
    """
    Simple arcade controller. Based on https://xiaoxiae.github.io/Robotics-Simplified-Website/drivetrain-control/arcade-drive/.
    """
    def __init__(self):
        TeleopController.__init__(self)
        
    def control(self, x: float, y: float) -> IWCMotors:
        """
        x: float: x position of the joystick controller
        y: float: y position of the joystick controller
        """
        x = -x
        maximum = max(abs(x), abs(y))
        total, diff = (y + x), (y - x)

        # set speed according to quadrant
        if y >= 0:
            if x >= 0:  # quadrant I
                left_wheels = maximum
                right_wheels = diff
            else:       # quadrant II
                left_wheels = total
                right_wheels = maximum
        else:
            if x >=0:   # quadrant IV
                left_wheels = total
                right_wheels = -maximum
            else:       # quadrant III
                left_wheels = -maximum
                right_wheels = diff

        # print('right_wheels', right_wheels, 'left_wheels', left_wheels)

        if left_wheels < -1.:
            left_wheels = -1.
        if right_wheels > 1.:
            right_wheels = 1.

        left_dir = self._check_dir(left_wheels)
        right_dir = self._check_dir(right_wheels)
        left_speed = self._check_speed(left_wheels)
        right_speed = self._check_speed(right_wheels)

        # Create IWC Message
        IWC_cmd_msg = IWCMotors()
        IWC_cmd_msg.right_front_speed = abs(right_speed)
        IWC_cmd_msg.right_front_dir = right_dir

        IWC_cmd_msg.right_middle_speed = abs(right_speed)
        IWC_cmd_msg.right_middle_dir = right_dir

        IWC_cmd_msg.right_rear_speed = abs(right_speed)
        IWC_cmd_msg.right_rear_dir = right_dir

        IWC_cmd_msg.left_front_speed = abs(left_speed)
        IWC_cmd_msg.left_front_dir = left_dir

        IWC_cmd_msg.left_middle_speed = abs(left_speed)
        IWC_cmd_msg.left_middle_dir = left_dir

        IWC_cmd_msg.left_rear_speed = abs(left_speed)
        IWC_cmd_msg.left_rear_dir = left_dir

        print(IWC_cmd_msg)

        # Return IWC cmds
        return IWC_cmd_msg

