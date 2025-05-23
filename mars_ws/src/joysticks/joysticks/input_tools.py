"""
This module contains classes for handling logic for various button, throttle, or axis inputs
found on XBOX controllers. These classes are designed to process input from the controller
and convert it into control signals for actuators or other systems.

Classes:
    InputButton: Handles logic for a button that is either on or off based on whether it is held down.
    InputToggle: Handles logic for a button that toggles between on and off states each time it is pressed.
    InputAxis: Handles logic for an axis input, converting its position to a control signal when outside a deadzone.
    InputThrottle: Handles logic for a throttle input, converting its position to a control signal when outside a deadzone.
"""
from sensor_msgs.msg import Joy

class InputButton:
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

class InputToggle(InputButton):
    '''When the button is pressed, the control will switch between the provided off and on states'''

    def __init__(self, button_index, control_on, control_off=0):
        super().__init__(button_index, control_on, control_off)
        self.prev_state = True # Handle edge case where user is depressing button on start

    def button_logic(self, msg: Joy):
        # When the button is pressed, toggle between on and off
        if msg.buttons[self.button_index] != self.prev_state and msg.buttons[self.button_index]:
            return not self.state
        else:
            return self.state
        
    def update(self, msg: Joy):
        self.state = self.button_logic(msg) # Check for toggle
        self.prev_state = self.state # Update the state cache
        return (self.control_on if self.state else self.control_off)

class InputAxis:
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

class InputThrottle:
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