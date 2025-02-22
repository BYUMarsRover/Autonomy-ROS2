#!/usr/bin/env python3
"""
PURPOSE: Recieve ROS commands and send commands to science Arduino via serial USB.

SUBSCRIBED TO:
/science_control
"""

import rclpy
from rclpy.node import Node
from rover_msgs.msg import ScienceActuatorControl, ScienceSensorValues
from std_msgs.msg import Bool
import serial
import struct

# This module uses the Science Module Serial Communication Protocol
# as definded in the BYU-Mars-Rover-Wiki
# https://github.com/BYUMarsRover/BYU-Mars-Rover-Wiki/blob/main/engineering/science/serial_comms_protocol.md

COMMAND_PACKET_HEADER = 0x53
COMMAND_PACKET_FOOTER = 0x4D
RESPONSE_PACKET_HEADER = 0x52
RESPONSE_PACKET_FOOTER = 0x46

MAXIMUM_PACKET_SIZE = 0xFF
FULL_STEAM_FORWARD = 0x7F
FULL_STEAM_BACKWARD = 0x80

UPDATE_PROBE_CONTROL_COMMAND_WORD = 0x85
UPDATE_AUGER_CONTROL_COMMAND_WORD = 0x86
UPDATE_DRILL_CONTROL_COMMAND_WORD = 0xAA

UPDATE_PRIMARY_CACHE_DOOR_CONTROL_COMMAND_WORD = 0x87
# UPDATE_SECONDARY_CACHE_DOOR_CONTROL_COMMAND_WORD = 0x88  #Unused secondary cache door, removed for 2024/2025 module design
UPDATE_SECONDARY_CACHE_CONTROL_COMMAND_WORD = 0x89

RESPONSE_ECHO_LEN_INDEX = 1
RESPONSE_ECHO_MESSAGE_START_INDEX = RESPONSE_ECHO_LEN_INDEX + 1
RESPONSE_ERROR_LEN_BASE_INDEX = 3
RESPONSE_ERROR_MESSAGE_START_BASE_INDEX = RESPONSE_ERROR_LEN_BASE_INDEX + 1
RESPONSE_FOOTER_BASE_INDEX = 4

GET_TEMP_COMMAND_WORD = 0x0F
GET_HUM_COMMAND_WORD = 0x10

PROBE_TOOL_INDEX = 0
AUGER_TOOL_INDEX = 1

SENSOR_ERR_CODE = -1

BAUD_RATE = 115200

def signed_to_unsigned(signed_value):
    if signed_value < 0:
        # Compute the two's complement representation
        unsigned_value = (1 << 8) + signed_value
    else:
        unsigned_value = signed_value
    return unsigned_value

class ScienceSerialInterface(Node):
    """Bridge ROS messaging and science arduino"""

    def __init__(self):
        super().__init__('science_serial_interface')

        try:
            self.arduino = serial.Serial("/dev/rover/scienceArduinoNano", BAUD_RATE)
            # self.arduino = serial.Serial("/dev/ttyUSB0", BAUD_RATE) # - used for testing off of rover
            self.get_logger().info("Serial port initialized")
            print("Serial port initialized")
        except Exception as e:
            print("Error: scienceArduinoNano not yet ready")
            print(str(e))
            sys.stdout.flush()
            self.get_logger().error("Error: scienceArduinoNano not yet ready")
            self.get_logger().error(str(e))
            rclpy.shutdown()
            exit(0)

        self.sub_science_serial_drill = self.create_subscription(ScienceActuatorControl, '/science_serial_drill', self.drill_control_callback, 10)
        self.sub_science_serial_probe = self.create_subscription(ScienceActuatorControl, '/science_serial_probe', self.probe_control_callback, 10)
        self.sub_science_serial_auger = self.create_subscription(ScienceActuatorControl, '/science_serial_auger', self.auger_control_callback, 10)
        self.sub_science_serial_primary_cache_door = self.create_subscription(ScienceActuatorControl, '/science_serial_primary_cache_door', self.primary_cache_door_control_callback, 10)
        self.sub_science_serial_secondary_cache = self.create_subscription(ScienceActuatorControl, '/science_serial_secondary_cache', self.secondary_cache_control_callback, 10)
        self.sub_science_serial_override = self.create_subscription(Bool, '/science_serial_override', self.set_override_bit_callback, 10)

        self.info_publisher = self.create_publisher(ScienceSensorValues, '/science_sensor_values', 10)

        self.create_timer(100e-3, self.read_serial) # 10 Hz
        self.create_timer(1, self.query_temperature) # 1 Hz
        self.create_timer(1, self.query_humidity) # 1 Hz

        # Caches for control purposes
        self.current_tool_index = 0

        # Data structure for reading packets
        self.read_queue = []

        # Caches for sensor values
        self.temperature = None
        self.humidity = None

        # State Variable
        self.override_bit = False

    def set_override_bit_callback(self, msg: Bool): #Control is an int8control = FULL_STEAM_FORWARD if self.secondary_cache_state == True else FULL_STEAM_BACKWARD
        print("Received override change")
        self.override_bit = msg.data

    def write_actuator_control(self, command_word, control): #Control is an int8control = FULL_STEAM_FORWARD if self.secondary_cache_state == True else FULL_STEAM_BACKWARD
        self.write_serial([command_word, 0x01, signed_to_unsigned(control)])

    def drill_control_callback(self, msg: ScienceActuatorControl):
        print("Doing drill control callback")
        self.write_actuator_control(UPDATE_DRILL_CONTROL_COMMAND_WORD, msg.control)
    
    def probe_control_callback(self, msg: ScienceActuatorControl):
        print("Doing probe linear actuator callback")
        self.write_actuator_control(UPDATE_PROBE_CONTROL_COMMAND_WORD, msg.control)

    def auger_control_callback(self, msg: ScienceActuatorControl):
        print("Doing auger linear actuator callback")
        self.write_actuator_control(UPDATE_AUGER_CONTROL_COMMAND_WORD, msg.control)

    def primary_cache_door_control_callback(self, msg: ScienceActuatorControl):
        print("Doing primary cache door callback")
        self.write_actuator_control(UPDATE_PRIMARY_CACHE_DOOR_CONTROL_COMMAND_WORD, msg.control)

    def secondary_cache_control_callback(self, msg: ScienceActuatorControl):
        print("Doing secondary cache door callback")
        self.write_actuator_control(UPDATE_SECONDARY_CACHE_CONTROL_COMMAND_WORD, msg.control)

    def write_serial(self, byte_array):
         # Configure a payload with the overhead formatting and send to arduino
        if len(byte_array) > MAXIMUM_PACKET_SIZE:
            # todo proper error message
            print(f"Provided command packet is of size {len(byte_array)}, maximum is {MAXIMUM_PACKET_SIZE}")
        elif self.arduino:
            byte_array[0] |= 0b01000000 if self.override_bit else 0b00000000  # Set the Override Bit
            byte_array.insert(0, COMMAND_PACKET_HEADER)
            byte_array.append(COMMAND_PACKET_FOOTER)
            self.arduino.write(struct.pack('B' * len(byte_array), *byte_array))
            hex_array = [ hex(x) for x in byte_array]
            print(f"Sending to arduino: {','.join(hex_array)}")
            # print(byte_array)
            
    def query_temperature(self):
        print('Doing query temp')
        # ask the arduino to return the temperature as a calibrated float
        self.write_serial([GET_TEMP_COMMAND_WORD, 0x00])

    def query_humidity(self):
        print('Doing query humidity')
        # ask the arduino to return the humidity as a calibrated float
        self.write_serial([GET_HUM_COMMAND_WORD, 0x00])

    def contains_possible_packet(self, queue):
        # Returns format information for a possible packet in the queue
        # Returns None, None if no possible packet in the queue

        temp = ",".join([ hex(x) for x in queue])
        # print(f"Looking for packet in [{temp}]")

        if len(queue) > RESPONSE_ECHO_LEN_INDEX:
            # The echo length should be loaded
            echo_length = queue[RESPONSE_ECHO_LEN_INDEX]

            if (len(queue) > RESPONSE_ERROR_LEN_BASE_INDEX + echo_length):
                # The error length should be loaded
                error_length = queue[RESPONSE_ERROR_LEN_BASE_INDEX + echo_length]

                if (len(queue) > RESPONSE_FOOTER_BASE_INDEX + echo_length + error_length):
                    # The footer byte should now be present
                    # print("Possible packet identified")
                    return (echo_length, error_length)

                else:
                    # print("Not long enough to contain footer length")
                    return (None, None)
            else:
                # print("Not long enough to contain error length")
                return (None, None)
        else:
            # print("Not long enough to contain echo length")
            return (None, None)

    def read_serial(self):
        # reads data off the buffer and trys to identify response packets

        if self.arduino.in_waiting > 0:
            # Attempt a read if there is something in the buffer
            # print(f"Arduino Buffer contains {self.arduino.in_waiting} bytes")

            for byte in self.arduino.read_all():
                # Load all bytes into the queue
                if len(self.read_queue) == 0:
                    # Nothing is in the queue, look for the header
                    if byte == RESPONSE_PACKET_HEADER:
                        # Found it, put it in the queue
                        self.read_queue.append(byte)
                    else:
                        # Not the header, move on to the next byte
                        continue
                else:
                    # The queue is already considering a possible packet, add this byte to the queue
                    self.read_queue.append(byte)

            # Begin searching the queue for packets
            packet_formatting = (0,0)
            while not None in (packet_formatting := self.contains_possible_packet(self.read_queue)):
                # The queue has a possible packet to investigate

                echo_length = packet_formatting[0]
                error_length = packet_formatting[1]

                if self.read_queue[RESPONSE_FOOTER_BASE_INDEX + echo_length + error_length] == RESPONSE_PACKET_FOOTER:
                    # This is a valid packet
                    self.process_packet(self.read_queue[0:(RESPONSE_FOOTER_BASE_INDEX + echo_length + error_length) + 1], packet_formatting)
                    self.read_queue = self.read_queue[(RESPONSE_FOOTER_BASE_INDEX + echo_length + error_length) + 1:]

                else:
                    # Bad packet, pop bytes until a new header byte is reached
                    # print("Bad packet")
                    while True:
                        self.read_queue = self.read_queue[1:]
                        if len(self.read_queue) == 0 or self.read_queue[0] == RESPONSE_PACKET_HEADER:
                            break

    def publish_sensors_value(self):
        if (self.temperature is None):
            self.temperature = SENSOR_ERR_CODE
        if (self.humidity is None):
            self.humidity = SENSOR_ERR_CODE
        self.info_publisher.publish(ScienceSensorValues(temperature=int(self.temperature),moisture=int(self.humidity)))

    def process_packet(self, response_packet, formatting_data):
        # Process a valid response packet
        echo_length = formatting_data[0]
        error_length = formatting_data[1]

        echo_message = response_packet[RESPONSE_ECHO_MESSAGE_START_INDEX : RESPONSE_ECHO_MESSAGE_START_INDEX + echo_length]
        error_code = response_packet[RESPONSE_ECHO_MESSAGE_START_INDEX + echo_length]
        error_message = response_packet[RESPONSE_ERROR_MESSAGE_START_BASE_INDEX + echo_length: RESPONSE_ERROR_MESSAGE_START_BASE_INDEX + echo_length + error_length]

        # print(f'Received Reponse Packet:\n\tEcho: [{",".join([ hex(x) for x in response_packet])}]\n\tError Code: {error_code}\n\tError Message: [{",".join([ hex(x) for x in error_message])}]')
        # print(f'ASCII: {bytes(error_message).decode()}')
        try:
            print(f'Received Response Packet:({error_code}) {bytes(error_message).decode()}')
        except:
            print(f'Received Response Packet with invalid ascii:({error_code}) {",".join([ hex(x) for x in error_message])}')

        function_address = echo_message[1] & 0b11111
        if function_address == (GET_TEMP_COMMAND_WORD & 0b11111):
            # This packet contains temperature data as a float
            self.temperature = struct.unpack('<f', struct.pack('B' * len(error_message), *error_message))[0] #For some reason struct.unpack returns a tuple with a single value. Da heck
            # print(f"Received temp: {self.temperature}")
            self.publish_sensors_value()
        elif function_address == (GET_HUM_COMMAND_WORD & 0b11111):
            # This packet contains humidity data as a float
            self.humidity = struct.unpack('<f', struct.pack('B' * len(error_message), *error_message))[0]
            # print(f"Received humidity: {self.humidity}")
            self.publish_sensors_value()

def main(args=None):
    rclpy.init(args=args)
    science_serial_interface = ScienceSerialInterface()

    science_serial_interface.get_logger().info('Science Serial Online')
    rclpy.spin(science_serial_interface)
    science_serial_interface.destroy_node()
    rclpy.shutdown

    if science_serial_interface.arduino:
        science_serial_interface.arduino.close()
        science_serial_interface.arduino = None

if __name__ == '__main__':
    main()