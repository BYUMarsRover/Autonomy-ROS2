#!/usr/bin/env python3
"""
PURPOSE: Receive ROS commands and send commands to the science Arduino via serial USB.

SUBSCRIBED TO:
/science_serial_probe
/science_serial_auger
/science_serial_primary_cache_door
/science_serial_secondary_cache_door
/science_serial_secondary_cache
/science_serial_drill
/science_serial_override
/science_serial_tx_request

PUBLISHED TO:
/science_serial_tx_notification
/science_serial_rx_notification
/science_serial_rx_packet

FUNCTIONALITY:
- Initializes a serial connection with the science Arduino.
- Sends actuator control commands to the Arduino based on ROS messages.
- Handles override bit changes for command packets.
- Reads and processes response packets from the Arduino.
- Publishes notifications and processed packets to ROS topics.
"""

import rclpy
import sys
from rclpy.node import Node
from rover_msgs.msg import ScienceActuatorControl, ScienceSensorValues, ScienceSerialTxPacket, ScienceSerialRxPacket
from std_msgs.msg import Bool, UInt8MultiArray, Empty, UInt16, Float32
import serial
import struct
import time
from science.function_mapping.function_map import ScienceModuleFunctionList as SMFL

# This module uses the Science Module Serial Communication Protocol
# as definded in the BYU-Mars-Rover-Wiki
# https://github.com/BYUMarsRover/BYU-Mars-Rover-Wiki/blob/main/engineering/science/serial_comms_protocol.md

COMMAND_PACKET_HEADER = 0x53
COMMAND_PACKET_FOOTER = 0x4D
RESPONSE_PACKET_HEADER = 0x52
RESPONSE_PACKET_FOOTER = 0x46

MAXIMUM_PACKET_SIZE = 0xFF

RESPONSE_ECHO_LEN_INDEX = 1
RESPONSE_ECHO_MESSAGE_START_INDEX = RESPONSE_ECHO_LEN_INDEX + 1
RESPONSE_ERROR_LEN_BASE_INDEX = 3
RESPONSE_ERROR_MESSAGE_START_BASE_INDEX = RESPONSE_ERROR_LEN_BASE_INDEX + 1
RESPONSE_FOOTER_BASE_INDEX = 4

PROBE_ACTUATOR_INDEX = 0
AUGER_ACTUATOR_INDEX = 1
PRIMARY_DOOR_ACTUATOR_INDEX = 2
SECONDARY_DOOR_ACTUATOR_INDEX = 3
SECONDARY_CACHE_ACTUATOR_INDEX = 4
DRILL_ACTUATOR_INDEX = 5

SENSOR_ERR_CODE = -1
BAUD_RATE = 115200

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

        self.create_subscription(ScienceActuatorControl, '/science_serial_probe',                lambda msg: self.actuator_control_callback(msg, PROBE_ACTUATOR_INDEX), 10)
        self.create_subscription(ScienceActuatorControl, '/science_serial_auger',                lambda msg: self.actuator_control_callback(msg, AUGER_ACTUATOR_INDEX), 10)
        self.create_subscription(ScienceActuatorControl, '/science_serial_primary_cache_door',   lambda msg: self.actuator_control_callback(msg, PRIMARY_DOOR_ACTUATOR_INDEX), 10)
        self.create_subscription(ScienceActuatorControl, '/science_serial_secondary_cache_door', lambda msg: self.actuator_control_callback(msg, SECONDARY_DOOR_ACTUATOR_INDEX), 10)
        self.create_subscription(ScienceActuatorControl, '/science_serial_secondary_cache',      lambda msg: self.actuator_control_callback(msg, SECONDARY_CACHE_ACTUATOR_INDEX), 10)
        self.create_subscription(ScienceActuatorControl, '/science_serial_drill',                lambda msg: self.actuator_control_callback(msg, DRILL_ACTUATOR_INDEX), 10)
        self.create_subscription(Bool, '/science_serial_override', self.set_override_bit_callback, 10)

        # Serial Communication Exchange
        self.sub_science_serial_tx_request = self.create_subscription(ScienceSerialTxPacket, '/science_serial_tx_request', self.perform_tx_request, 10)
        self.pub_science_serial_tx_notification = self.create_publisher(UInt8MultiArray, '/science_serial_tx_notification', 10)
        self.pub_science_serial_rx_notification = self.create_publisher(UInt8MultiArray, '/science_serial_rx_notification', 10)
        self.pub_science_serial_rx_packet = self.create_publisher(ScienceSerialRxPacket, '/science_serial_rx_packet', 10)

        # Timer to read the serial input
        self.create_timer(100e-3, self.read_serial) # 10 Hz

        # Data structure for reading packets
        self.read_queue = []

        # State Variable
        self.override_bit = False

    # Callbacks for Subscribers

    def actuator_control_callback(self, msg: ScienceActuatorControl, sensor_index):
        self.perform_tx_request(SMFL.get_tx_get_update_actuator_control(sensor_index, msg.control))

    def set_override_bit_callback(self, msg: Bool):
        print("Received override change")
        self.override_bit = msg.data

    # Publishing for RXTX Monitoring

    def publish_serial_tx_notification(self, data):
        msg = UInt8MultiArray()
        msg.data = data
        self.pub_science_serial_tx_notification.publish(msg)

    def publish_serial_rx_notification(self, data):
        msg = UInt8MultiArray()
        msg.data = data
        self.pub_science_serial_rx_notification.publish(msg)

    # Writing to the Serial Buses

    # Sends published serial packets from the GUI to the arduino
    def perform_tx_request(self, msg: ScienceSerialTxPacket):
        if len(msg.packet) > 0:
            # If the packet is not empty, just send it to the arduino
            self.write_serial(msg.packet)
        else:
            # If the packet is empty, build from command word and operands
            self.author_packet([msg.command_word, len(msg.operands)] + list(msg.operands), self.override_bit)

    def write_serial(self, packet):
        self.arduino.write(struct.pack('B' * len(packet), *packet))
        self.publish_serial_tx_notification(packet)

    def author_packet(self, byte_array, override):
         # Configure a payload with the overhead formatting and send to arduino
        if len(byte_array) > MAXIMUM_PACKET_SIZE:
            # todo proper error message
            print(f"Provided command packet is of size {len(byte_array)}, maximum is {MAXIMUM_PACKET_SIZE}")
        elif self.arduino:
            byte_array[0] |= 0b01000000 if override else 0b00000000  # Set the Override Bit
            byte_array.insert(0, COMMAND_PACKET_HEADER)
            byte_array.append(COMMAND_PACKET_FOOTER)
            self.write_serial(byte_array)

            # hex_array = [ hex(x) for x in byte_array]
            # print(f"Sending to arduino: {','.join(hex_array)}")

    # Reading in from the Serial Bus

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

            buffer_contents = self.arduino.read_all()
            self.publish_serial_rx_notification(buffer_contents)
            for byte in buffer_contents:
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

        # Publish packet notification to ROS
        rx_packet = ScienceSerialRxPacket(
            timestamp = time.time(),  # Timestamp as a float with sub-second precision
            echo = echo_message,  # Random echo bytes
            error_code = error_code,
            message = error_message
        )
        self.pub_science_serial_rx_packet.publish(rx_packet)

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