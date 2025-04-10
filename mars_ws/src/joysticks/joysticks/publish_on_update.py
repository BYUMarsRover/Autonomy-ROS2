"""
This module contains classes for managing and publishing updates to ROS messages
only when the control values have changed. This is useful for reducing unnecessary
message traffic in a ROS system by ensuring that messages are only published when
there are actual changes in the data.

Classes:
    UpdatePublisher: Manages a single ROS message publisher and only publishes
                     updates when the control values have changed.
    UpdatePublisherGroup: Manages a group of UpdatePublisher instances and publishes
                          updates for all of them.
    ScienceActuatorPublisher: A specialized UpdatePublisher for tracking and publishing
                              changes to an actuator control value, expecting a value
                              between -128 and 127.
    ElevatorPublisher: A specialized UpdatePublisher for tracking and publishing
                              changes to the elevator
"""

import importlib

class UpdatePublisher():
    '''Only publishes new data if the control value has been changed'''

    def __init__(self, pub, msg_type_str, attr_str_list=['data']):
        self.pub = pub

        # Dynamically import the message type
        module_name, class_name = msg_type_str.rsplit('.', 1)
        self.msg_module = importlib.import_module(module_name)
        self.msg_type = getattr(self.msg_module, class_name)

        # Set up state varibles
        self.attr_str_list = attr_str_list
        self.values = [False for x in range(len(attr_str_list))]
        self.is_updated = False

    def set(self, new_value, attr_str='data'):
        try:
            attr_index = self.attr_str_list.index(attr_str)
            self.is_updated = self.values[attr_index] != new_value # Set this flag if this attribute has updated
            self.values[attr_index] = new_value
        except ValueError:
            pass

    def publish(self):
        if (self.is_updated):
            msg = self.msg_type() # Make Message Object
            for i in range(len(self.attr_str_list)):
                setattr(msg, self.attr_str_list[i], self.values[i]) # Fill out the attribute field
            self.pub.publish(msg)
            self.is_updated = False

class UpdatePublisherGroup():
    '''Handles a group of Update publishers'''

    def __init__(self):
        self.publisher_list = []

    def add_publisher(self, publisher: UpdatePublisher):
        self.publisher_list.append(publisher)
        return publisher

    def publish(self):
        for publisher in self.publisher_list:
            publisher.publish()


class ScienceActuatorPublisher(UpdatePublisher):
    '''Tracks the control for an actuator and publishes the change.
    Expects a value between -128 and 127'''

    def __init__(self, pub, format_func=None):
        super().__init__(pub, 'rover_msgs.msg.ScienceActuatorControl', ['control'])
        self.format_func = format_func

    def set(self, new_control):
        if self.format_func != None:
            super().set(self.format_func(new_control), 'control')
        else:
            super().set(int(min(max(new_control, -128), 127)), 'control')

    def convert_axis_to_unsigned_integer(axis_val):
        '''Maps from [-1,1] to [-127, 127]'''
        return int(axis_val * 127)

class ElevatorPublisher(UpdatePublisher):
    '''Updates the control for the elevator'''

    def __init__(self, pub):
        super().__init__(pub, 'rover_msgs.msg.Elevator', ['elevator_direction', 'elevator_speed'])

    def set_dir(self, dir: int):
        super().set(dir, 'elevator_direction')

    def set_speed(self, speed: int):
        super().set(speed, 'elevator_speed')