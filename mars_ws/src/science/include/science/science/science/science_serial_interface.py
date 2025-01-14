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
COMMAND_PACKET_DELIMITER = 0x80

TEMPERATURE_DIVISOR = -9.34579
TEMPERATURE_CONSTANT = 80.3
MOISTURE_DIVISOR = -6.5
MOISTURE_CONSTANT = 154.0

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
        self.auger_speed = msg.auger_speed
    
    def linear_actuator_callback(self, msg: ScienceLinearActuatorDirection):
        self.linear_actuator_speed = msg.direction

    def primary_cache_door_callback(self, msg: ScienceCacheDoor):
        self.primary_cache_door_position = msg.position

    def secondary_cache_door_callback(self, msg: ScienceCacheDoor):
        self.secondary_cache_door_position = msg.position

    def tool_position_callback(self, msg: ScienceToolPosition):
        self.current_tool_index = msg.position

    def secondary_cache_position_callback(self, msg: ScienceSecondaryCachePosition):
        self.secondary_cache_position = msg.secondary_cache_position

    # Callback function for when a science message gets published
    def write_serial(self):
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