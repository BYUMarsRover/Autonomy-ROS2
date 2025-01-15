#!/usr/bin/env python3
"""
Test Controller Setup for Science Arduino

NOTE: This is the most up to date off-rover science module test.
Requires a USB connection to the science module Arduino, and
a USB connection the science module Xbox controller.

Last Updated: January 2025

"""

import time
from inputs import get_gamepad
import math
import threading
import serial
import struct

class TestController(object):
    MAX_TRIG_VAL = math.pow(2, 8)
    MAX_JOY_VAL = math.pow(2, 15)

    def __init__(self, precision=4, deadzone=.09):

        self._precision = precision
        self.deadzone = deadzone

        self._LeftJoystickY = 0.0
        self._LeftJoystickX = 0.0
        self._RightJoystickY = 0.0
        self._RightJoystickX = 0.0
        self._LeftTrigger = 0.0
        self._RightTrigger = 0.0
        self._LeftBumper = 0.0
        self._RightBumper = 0.0
        self._A = 0.0
        self._X = 0.0
        self._Y = 0.0
        self._B = 0.0
        self._LeftThumb = 0.0
        self._RightThumb = 0.0
        self._Back = 0.0
        self._Start = 0.0
        self._HorzDPad = 0.0
        self._VertDPad = 0.0

        self._monitor_thread = threading.Thread(target=self._monitor_controller, args=())
        self._monitor_thread.daemon = True
        self._monitor_thread.start()

        self.primary_cache_door = False
        self.secondary_cache_door = False
        self.secondary_cache_engage = False
        self.switch_auger_direction = 0
        self.auger_position: int = 0
        self.linear_actuator_speed: int = 0
        self.elevator_direction = 0
        self.auger_speed: int = 0

        self.arduino = serial.Serial("/dev/ttyUSB0", 9600)

        # constants
        self.PACKET_DELIMITER = 0x80

    @property
    def LeftJoystickY(self):
        if abs(self._LeftJoystickY) < self.deadzone:
            return 0.
        return round(self._LeftJoystickY, self._precision)
    @property
    def LeftJoystickX(self):
        if abs(self._LeftJoystickX) < self.deadzone:
            return 0.
        return round(self._LeftJoystickX, self._precision)
    @property
    def RightJoystickY(self):
        if abs(self._RightJoystickY) < self.deadzone:
            return 0.
        return round(self._RightJoystickY, self._precision)
    @property
    def RightJoystickX(self):
        if abs(self._RightJoystickX) < self.deadzone:
            return 0.
        return round(self._RightJoystickX, self._precision)
    @property
    def LeftTrigger(self):
        if abs(self._LeftTrigger) < self.deadzone:
            return 0.
        return round(self._LeftTrigger, self._precision)
    @property
    def RightTrigger(self):
        if abs(self._RightTrigger) < self.deadzone:
            return 0.
        return round(self._RightTrigger, self._precision)
    @property
    def LeftBumper(self):
        if abs(self._LeftBumper) < self.deadzone:
            return 0.
        return round(self._LeftBumper, self._precision)
    @property
    def RightBumper(self):
        if abs(self._RightBumper) < self.deadzone:
            return 0.
        return round(self._RightBumper, self._precision)
    @property
    def A(self):
        if abs(self._A) < self.deadzone:
            return 0.
        return round(self._A, self._precision)
    @property
    def X(self):
        if abs(self._X) < self.deadzone:
            return 0.
        return round(self._X, self._precision)
    @property
    def Y(self):
        if abs(self._Y) < self.deadzone:
            return 0.
        return round(self._Y, self._precision)
    @property
    def B(self):
        if abs(self._B) < self.deadzone:
            return 0.
        return round(self._B, self._precision)
    @property
    def LeftThumb(self):
        if abs(self._LeftThumb) < self.deadzone:
            return 0.
        return round(self._LeftThumb, self._precision)
    @property
    def RightThumb(self):
        if abs(self._RightThumb) < self.deadzone:
            return 0.
        return round(self._RightThumb, self._precision)
    @property
    def Back(self):
        if abs(self._Back) < self.deadzone:
            return 0.
        return round(self._Back, self._precision)
    @property
    def Start(self):
        if abs(self._Start) < self.deadzone:
            return 0.
        return round(self._Start, self._precision)
    @property
    def HorzDPad(self):
        if abs(self._HorzDPad) < self.deadzone:
            return 0.
        return round(self._HorzDPad, self._precision)
    @property
    def VertDPad(self):
        if abs(self._VertDPad) < self.deadzone:
            return 0.
        return round(self._VertDPad, self._precision)

    def readAll(self):
        return [self.LeftJoystickY, 
                self.LeftJoystickX, 
                self.RightJoystickY, 
                self.RightJoystickX, 
                self.LeftTrigger, 
                self.RightTrigger, 
                self.LeftBumper, 
                self.RightBumper, 
                self.A, 
                self.Y, 
                self.X, 
                self.B, 
                self.LeftThumb, 
                self.RightThumb, 
                self.Back, 
                self.Start, 
                self.HorzDPad, 
                self.VertDPad]


    def _monitor_controller(self):
        while True:
            events = get_gamepad()
            for event in events:
                if event.code == 'ABS_Y':
                    self._LeftJoystickY = event.state / TestController.MAX_JOY_VAL # normalize between -1 and 1
                elif event.code == 'ABS_X':
                    self._LeftJoystickX = event.state / TestController.MAX_JOY_VAL # normalize between -1 and 1
                elif event.code == 'ABS_RY':
                    self._RightJoystickY = event.state / TestController.MAX_JOY_VAL # normalize between -1 and 1
                elif event.code == 'ABS_RX':
                    self._RightJoystickX = event.state / TestController.MAX_JOY_VAL # normalize between -1 and 1
                elif event.code == 'ABS_Z':
                    self._LeftTrigger = event.state / TestController.MAX_TRIG_VAL # normalize between 0 and 1
                elif event.code == 'ABS_RZ':
                    self._RightTrigger = event.state / TestController.MAX_TRIG_VAL # normalize between 0 and 1
                elif event.code == 'BTN_TL':
                    self._LeftBumper = event.state
                elif event.code == 'BTN_TR':
                    self._RightBumper = event.state
                elif event.code == 'BTN_SOUTH':
                    self._A = event.state
                elif event.code == 'BTN_NORTH':
                    self._Y = event.state 
                elif event.code == 'BTN_WEST':
                    self._X = event.state 
                elif event.code == 'BTN_EAST':
                    self._B = event.state
                elif event.code == 'BTN_THUMBL':
                    self._LeftThumb = event.state
                elif event.code == 'BTN_THUMBR':
                    self._RightThumb = event.state
                elif event.code == 'BTN_SELECT':
                    self._Back = event.state
                elif event.code == 'BTN_START':
                    self._Start = event.state
                elif event.code == 'ABS_HAT0X':
                    self._HorzDPad = event.state
                elif event.code == 'ABS_HAT0Y':
                    self._VertDPad = -event.state
        
    def isActive(self):
        return True

    def parseCommands(self):
        DEAD_ZONE = 0.05

        linear_actuator_speed = 0.0
        elev_direction = 0
        auger_speed = 0.0

        if self.RightJoystickY > DEAD_ZONE:
            linear_actuator_speed = self.RightJoystickY
        elif self.RightJoystickY < -DEAD_ZONE:
            linear_actuator_speed = self.RightJoystickY
        else:
            linear_actuator_speed = 0

        # Elevator Control: LEFT JOYSTICK VERTICAL
        if self.LeftJoystickY > DEAD_ZONE:
            elev_direction = 1
        elif self.LeftJoystickY < -DEAD_ZONE:
            elev_direction = -1
        else:
            elev_direction = 0

        # Primary Cache Door: A BUTTON
        self.primary_cache_door = self.A
        # Secondary Cache Door: B Button
        self.secondary_cache_door = self.B
        # Secondary Cache Engage: Start Button
        self.secondary_cache_engage = not(self.Start)

        if self.RightTrigger > DEAD_ZONE:
            auger_speed = self.RightTrigger
        elif self.LeftTrigger > DEAD_ZONE:
            auger_speed = -self.LeftTrigger
        else:
            auger_speed = 0

        if self.LeftBumper:
            switch_auger_direction = -1
        elif self.RightBumper:
            switch_auger_direction = 1
        else:
            switch_auger_direction = 0

        if elev_direction != self.elevator_direction:
            self.elevator_direction = elev_direction
            # print('MOVING ELEVATOR', self.elevator_direction)

        if linear_actuator_speed != self.linear_actuator_speed:
            self.linear_actuator_speed = int(linear_actuator_speed * -127)
            # print('MOVING LINEAR ACTUATOR', self.linear_actuator_speed)

        if auger_speed != self.auger_speed:
            self.auger_speed = int(auger_speed * 127)
            # print('auger_speed', self.auger_speed)

        if switch_auger_direction != self.switch_auger_direction:
            self.switch_auger_direction = switch_auger_direction
            if self.switch_auger_direction != 0:
                # print('switching auger', self.switch_auger_direction)
                self.auger_position = (self.auger_position + self.switch_auger_direction) % 2

    def write_serial(self):
        if self.arduino:
            message_bytes = [
                self.PACKET_DELIMITER,
                signed_to_unsigned(self.auger_position),
                signed_to_unsigned(self.linear_actuator_speed),
                signed_to_unsigned(self.auger_speed),
                signed_to_unsigned(0 if self.primary_cache_door else 1),
                signed_to_unsigned(0 if self.secondary_cache_door else 1),
                signed_to_unsigned(0 if self.secondary_cache_engage else 1)
            ]
            print('Sending:', message_bytes)
            # Write out the string to the arduino
            self.arduino.write(struct.pack('B' * len(message_bytes), *message_bytes))

def signed_to_unsigned(signed_value):
    if signed_value < 0:
        # Compute the two's complement representation
        unsigned_value = (1 << 8) + signed_value
    else:
        unsigned_value = signed_value
    return unsigned_value

if __name__ == '__main__':
    joy = TestController()
    while joy.isActive():
        joy.parseCommands()
        joy.write_serial()
        time.sleep(.1)