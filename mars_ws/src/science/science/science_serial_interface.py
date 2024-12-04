#!/usr/bin/env python3
"""
PURPOSE: Recieve ROS commands and send commands to science Arduino via serial USB.

SUBSCRIBED TO:
/science_control
"""

import rclpy
from rclpy.node import Node
from rover_msgs.msg import ScienceAugerOn, ScienceToolPosition, ScienceLinearActuatorDirection, ScienceCacheDoor, ScienceSensorValues, ScienceSecondaryCachePosition
import serial
import struct

CACHE_DOOR_OPEN = False
CACHE_DOOR_CLOSED = True
SERIAL_CACHE_DOOR_OPEN = 0
SERIAL_CACHE_DOOR_CLOSED = 1
CACHE_ENGAGED = 1
CACHE_DISENGAGED = 0

TEMPERATURE_DIVISOR = -9.34579
TEMPERATURE_CONSTANT = 80.3
MOISTURE_DIVISOR = -6.5
MOISTURE_CONSTANT = 154.0

COMMAND_PACKET_HEADER = 0x53
COMMAND_PACKET_FOOTER = 0x42

MAXIMUM_PACKET_SIZE = 0xFF
FULL_STEAM_FORWARD = 0x79
FULL_STEAM_BACKWARD = 0x80

UPDATE_PROBE_CONTROL_COMMAND_WORD = 0x85
UPDATE_AUGER_CONTROL_COMMAND_WORD = 0x86
UPDATE_DRILL_CONTROL_COMMAND_WORD = 0x8A

UPDATE_PRIMARY_CACHE_DOOR_CONTROL_COMMAND_WORD = 0x87
UPDATE_SECONDARY_CACHE_DOOR_CONTROL_COMMAND_WORD = 0x88
UPDATE_SECONDARY_CACHE_CONTROL_COMMAND_WORD = 0x89

PROBE_TOOL_INDEX = 0
AUGER_TOOL_INDEX = 1

def signed_to_unsigned(signed_value):
    if signed_value < 0:
        # Compute the two's complement representation
        unsigned_value = (1 << 8) + signed_value
    else:
        unsigned_value = signed_value
    return unsigned_value

def convertToCelsius(raw_value):
    print('Converting to celsius', raw_value)
    celsius = (raw_value / TEMPERATURE_DIVISOR) + TEMPERATURE_CONSTANT
    print('converted value (normal, int):', celsius, int(celsius))

    return int(celsius)

def calcMoisturePercentage(raw_value):
    print('Calculating Percentage', raw_value)
    moist = 522625.0 * raw_value**(-1.8)
    print('converted value (normal, int):', moist, int(moist))

    return int(moist)

class ScienceSerialInterface(Node):
    """Bridge ROS messaging and science arduino"""

    def __init__(self):
        super().__init__('science_serial_interface')

        self.science_serial_auger = self.create_subscription(ScienceAugerOn, '/science_serial_auger', self.auger_on_callback, 10)
        self.science_tool_position = self.create_subscription(ScienceToolPosition, '/science_tool_position', self.tool_position_callback, 10)
        self.science_serial_la = self.create_subscription(ScienceLinearActuatorDirection, '/science_serial_la', self.linear_actuator_callback, 10)
        self.science_serial_primary_cache_door = self.create_subscription(ScienceCacheDoor, '/science_serial_primary_cache_door', self.primary_cache_door_callback, 10)
        self.science_serial_secondary_cache_door = self.create_subscription(ScienceCacheDoor, '/science_serial_secondary_cache_door', self.secondary_cache_door_callback, 10)
        self.science_serial_secondary_cache = self.create_subscription(ScienceSecondaryCachePosition, '/science_serial_secondary_cache', self.secondary_cache_position_callback, 10)

        self.current_tool_index = 0
        self.linear_actuator_speed = 0
        self.auger_speed = 0
        self.primary_cache_door_position = CACHE_DOOR_CLOSED
        self.secondary_cache_door_position = CACHE_DOOR_CLOSED
        self.secondary_cache_position = CACHE_DISENGAGED

        self.info_publisher = self.create_publisher(ScienceSensorValues, '/science_sensor_values', 10)
        # self.arduino = serial.Serial("/dev/rover/scienceArduinoNano", 9600)
        self.arduino = serial.Serial("/dev/ttyS3",9600)

        hz = 10 # Rate of 10 Hz
        self.create_timer(1./hz, self.read_serial)
        self.create_timer(1./hz, self.write_serial)


    def auger_on_callback(self, msg: ScienceAugerOn):
        write_serial([UPDATE_DRILL_CONTROL_COMMAND_WORD, 0x01, msg.auger_speed])
    
    def linear_actuator_callback(self, msg: ScienceLinearActuatorDirection):
        if current_tool_index == PROBE_TOOL_INDEX: # Update Probe
            write_serial([UPDATE_PROBE_CONTROL_COMMAND_WORD, 0x01, msg.direction])
        else if current_tool_index == AUGER_TOOL_INDEX: # Update Auger
            write_serial([UPDATE_AUGER_CONTROL_COMMAND_WORD, 0x01, msg.direction])

    def primary_cache_door_callback(self, msg: ScienceCacheDoor):
        if msg.position == True: # Extend the trapdoor
            write_serial([UPDATE_PRIMARY_CACHE_DOOR_CONTROL_COMMAND_WORD, 0x01, FULL_STEAM_FORWARD])
        else: # Close the trapdoor
            write_serial([UPDATE_PRIMARY_CACHE_DOOR_CONTROL_COMMAND_WORD, 0x01, FULL_STEAM_BACKWARD])


    def secondary_cache_door_callback(self, msg: ScienceCacheDoor):
        if msg.position == True: # Extend the trapdoor
            write_serial([UPDATE_SECONDARY_CACHE_DOOR_CONTROL_COMMAND_WORD, 0x01, FULL_STEAM_FORWARD])
        else: # Close the trapdoor
            write_serial([UPDATE_SECONDARY_CACHE_DOOR_CONTROL_COMMAND_WORD, 0x01, FULL_STEAM_BACKWARD])

    def tool_position_callback(self, msg: ScienceToolPosition):
        self.current_tool_index = msg.position

    def secondary_cache_position_callback(self, msg: ScienceSecondaryCachePosition):
        if msg.secondary_cache_position > 0: # Extend the cache
            write_serial([UPDATE_SECONDARY_CACHE_CONTROL_COMMAND_WORD, 0x01, FULL_STEAM_FORWARD])
        else: # Close the cache
            write_serial([UPDATE_SECONDARY_CACHE_CONTROL_COMMAND_WORD, 0x01, FULL_STEAM_BACKWARD])

    # Configure a payload with the overhead formatting and send to arduino
    def write_serial(self, byte_array):
        if len(byte_array) > MAXIMUM_PACKET_SIZE:
            # todo proper error message
            print(f"Provided command packet is of size {len(byte_array)}, maximum is {MAXIMUM_PACKET_SIZE}")
        else if self.arduino:
            byte_array.insert(0, COMMAND_PACKET_HEADER)
            byte_array.append(COMMAND_PACKET_FOOTER)
            self.arduino.write(struct.pack('B' * len(byte_array), *byte_array))


    # Callback function for when a science message gets published
    def write_serial_old(self):
        if self.arduino:
            message_bytes = [
                COMMAND_PACKET_DELIMITER,
                signed_to_unsigned(self.current_tool_index),
                signed_to_unsigned(self.linear_actuator_speed),
                signed_to_unsigned(self.auger_speed),
                signed_to_unsigned(SERIAL_CACHE_DOOR_CLOSED if self.primary_cache_door_position else SERIAL_CACHE_DOOR_OPEN),
                signed_to_unsigned(SERIAL_CACHE_DOOR_CLOSED if self.secondary_cache_door_position else SERIAL_CACHE_DOOR_OPEN),
                signed_to_unsigned(self.secondary_cache_position)
            ]
            #print('Sending:', message_bytes)
            # Write out the string to the arduino
            self.arduino.write(struct.pack('B' * len(message_bytes), *message_bytes))


    def read_serial(self):
        # only attempt reads when there are available bytes on the Serial input buffer
        if self.arduino.in_waiting > 0:
            # discard a line to reset to correct position
            self.arduino.readline()
            # get data until a '\n' character (blocking), decode it, strip off the \n, then split it by :
            data = self.arduino.readline()
            data = data.decode('utf-8').strip().split(':')
            data = [int(n) for n in data]
            self.info_publisher.publish(ScienceSensorValues(convertToCelsius(data[0]), calcMoisturePercentage(data[1])))

def main(args=None):
    rclpy.init(args=args)
    science_serial_interface = ScienceSerialInterface()
    # set loop rate
    # rate = science_serial_interface.create_rate(10) #Rate of 10 Hz
    
    # while not science_serial_interface.arduino:
    #     pass

    science_serial_interface.get_logger().info('Science Serial Online')
    rclpy.spin(science_serial_interface)
    science_serial_interface.destroy_node()
    rclpy.shutdown

    if science_serial_interface.arduino:
        science_serial_interface.arduino.close()
        science_serial_interface.arduino = None

if __name__ == '__main__':
    main()