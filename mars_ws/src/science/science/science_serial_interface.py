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

# This module uses the Science Module Serial Communication Protocol
# as definded in the BYU-Mars-Rover-Wiki
# https://github.com/BYUMarsRover/BYU-Mars-Rover-Wiki/blob/main/engineering/science/serial_comms_protocol.md

COMMAND_PACKET_HEADER = 0x53
COMMAND_PACKET_FOOTER = 0x42
RESPONSE_PACKET_HEADER = 0x52
RESPONSE_PACKET_FOOTER = 0x46

MAXIMUM_PACKET_SIZE = 0xFF
FULL_STEAM_FORWARD = 0x7F
FULL_STEAM_BACKWARD = 0x80

UPDATE_PROBE_CONTROL_COMMAND_WORD = 0x85
UPDATE_AUGER_CONTROL_COMMAND_WORD = 0x86
UPDATE_DRILL_CONTROL_COMMAND_WORD = 0x8A

UPDATE_PRIMARY_CACHE_DOOR_CONTROL_COMMAND_WORD = 0x87
UPDATE_SECONDARY_CACHE_DOOR_CONTROL_COMMAND_WORD = 0x88
UPDATE_SECONDARY_CACHE_CONTROL_COMMAND_WORD = 0x89

RESPONSE_ECHO_LEN_INDEX = 1
RESPONSE_ECHO_MESSGAE_START_INDEX = 2
RESPONSE_ERROR_LEN_BASE_INDEX = 3
RESPONSE_ERROR_MESSGAE_START_BASE_INDEX = 4
RESPONSE_FOOTER_BASE_INDEX = 5

GET_TEMP_COMMAND_WORD = 0x0F
GET_HUM_COMMAND_WORD = 0x10

PROBE_TOOL_INDEX = 0
AUGER_TOOL_INDEX = 1

BAUD_RATE = 115200

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

        self.info_publisher = self.create_publisher(ScienceSensorValues, '/science_sensor_values', 10)
        # self.arduino = serial.Serial("/dev/rover/scienceArduinoNano", BAUD_RATE)
        self.arduino = serial.Serial("/dev/ttyS3", BAUD_RATE)

        hz = 10 # Rate of 10 Hz
        self.create_timer(1./hz, self.query_temperature)
        self.create_timer(1./hz, self.query_humidity)
        self.create_timer(1./hz, self.read_serial)

        # Caches for control purposes
        self.current_tool_index = 0

        # Data structure for reading packets
        self.read_queue = []

        # Caches for sensor values
        self.temperature = None
        self.humidity = None


    def auger_on_callback(self, msg: ScienceAugerOn):
        write_serial([UPDATE_DRILL_CONTROL_COMMAND_WORD, 0x01, msg.auger_speed])
    
    def linear_actuator_callback(self, msg: ScienceLinearActuatorDirection):
        if current_tool_index == PROBE_TOOL_INDEX: # Update Probe
            write_serial([UPDATE_PROBE_CONTROL_COMMAND_WORD, 0x01, msg.direction])
        elif current_tool_index == AUGER_TOOL_INDEX: # Update Auger
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

    def write_serial(self, byte_array):
         # Configure a payload with the overhead formatting and send to arduino
        if len(byte_array) > MAXIMUM_PACKET_SIZE:
            # todo proper error message
            print(f"Provided command packet is of size {len(byte_array)}, maximum is {MAXIMUM_PACKET_SIZE}")
        elif self.arduino:
            byte_array.insert(0, COMMAND_PACKET_HEADER)
            byte_array.append(COMMAND_PACKET_FOOTER)
            self.arduino.write(struct.pack('B' * len(byte_array), *byte_array))
            
    def query_temperature(self):
        # ask the arduino to return the temperature as a calibrated float
        write_serial([GET_TEMP_COMMAND_WORD, 0x00])

    def query_humidity(self):
        # ask the arduino to return the humidity as a calibrated float
        write_serial([GET_HUM_COMMAND_WORD, 0x00])

    def contains_possible_packet(self, queue):
        # Returns format information for a possible packet in the queue
        # Returns None, None if no possible packet in the queue

        if len(queue) > RESPONSE_ECHO_LEN_BASE_INDEX:
            # The echo length should be loaded
            echo_length = queue[RESPONSE_ECHO_LEN_INDEX]

            if (len(queue) > RESPONSE_ERROR_LEN_BASE_INDEX + echo_length):
                # The error length should be loaded
                error_length = queue[RESPONSE_ERROR_LEN_BASE_INDEX + echo_length]

                if (len(queue) > RESPONSE_FOOTER_BASE_INDEX + echo_length + error_length):
                    # The footer byte should now be present
                    return echo_length, error_length

                else:
                    return None, None
            else:
                return None, None
        else:
            return None, None

    def read_serial(self):
        # reads data off the buffer and trys to identify response packets

        if self.arduino.in_waiting > 0:
            # Attempt a read if there is something in the buffer

            for byte in self.arduino.read_all():
                # Load all bytes into the queue
                if len(read_queue) == 0:
                    # Nothing is in the queue, look for the header
                    if byte == RESPONSE_PACKET_HEADER:
                        # Found it, put it in the queue
                        read_queue.append(byte)
                    else:
                        # Not the header, move on to the next byte
                        continue
                else:
                    # The queue is already considering a possible packet, add this byte to the queue
                    read_queue.append(byte)

            # Begin searching the queue for packets
            while None in ((echo_length, error_length) := self.queue_contains_possible_packet(read_queue)):
                # The queue has a possible packet to investigate

                if read_queue[RESPONSE_FOOTER_BASE_INDEX + echo_length + error_length] == RESPONSE_PACKET_FOOTER:
                    # This is a valid packet
                    self.process_packet(read_queue[0:(RESPONSE_FOOTER_BASE_INDEX + echo_length + error_length)])

                else:
                    # Bad packet, pop bytes until a new header byte is reached
                    while read_queue[0] != RESPONSE_PACKET_HEADER:
                        read_queue.pop()

    def publish_sensors_value(self):
        self.info_publisher.publish(ScienceSensorValues(self.temperature, self.humidity))

    def process_packet(self, response_packet):
        # Process a valid response packet
        echo_length, error_length = self.queue_contains_possible_packet(response_packet)

        echo_message = response_packet[RESPONSE_ECHO_MESSGAE_START_INDEX : RESPONSE_ECHO_MESSGAE_START_INDEX + echo_length]
        error_code = response_packet[RESPONSE_ECHO_MESSGAE_START_INDEX + echo_length]
        error_message = response_packet[RESPONSE_ERROR_MESSGAE_START_BASE_INDEX + echo_length: RESPONSE_ERROR_MESSGAE_START_BASE_INDEX + echo_length + error_length]

        print(f'Received Reponse Packet:\n\tEcho: {echo_message}\n\tError Code: {error_code}\n\tError Message: {error_message}')

        match (echo_message[1] & 0b11111): # Check the function address
            case (GET_TEMP_COMMAND_WORD & 0b11111):
                # This packet contains temperature data as a float
                self.temperature = struct.unpack('<f', error_message)
            case (GET_HUM_COMMAND_WORD & 0b11111):
                # This packet contains temperature data as a float
                self.humidity = struct.unpack('<f', error_message)

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