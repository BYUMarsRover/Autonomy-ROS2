#!/usr/bin/env python3
"""
PURPOSE: Receive ROS commands and send commands to the science Arduino via serial USB.

SUBSCRIBED TO:
/science/serial/probe
/science/serial/auger
/science/serial/primary_cache_door
/science/serial/secondary_cache_door
/science/serial/secondary_cache
/science/serial/drill
/science/serial/override
/science/serial/tx_request

PUBLISHED TO:
/science/serial/tx_notification
/science/serial/rx_notification
/science/serial/rx_packet

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
from rover_msgs.msg import ScienceActuatorControl, ScienceSerialTxPacket, ScienceSerialRxPacket
from std_msgs.msg import Bool, UInt8MultiArray, Empty, String, Float32
import serial
import struct
import time
from science.function_mapping.function_map import ScienceModuleFunctionList as SMFL
from science.function_mapping.function_map import ScienceModuleFunctionListBuilder as SMFL_Builder

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

TOTAL_ACTUATORS = 6
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

        # Begin serial interface
        self.arduino = None
        self.establish_serial_connection()

        self.create_subscription(ScienceActuatorControl, '/science/serial/probe',                lambda msg: self.actuator_control_callback(msg, PROBE_ACTUATOR_INDEX), 10)
        self.create_subscription(ScienceActuatorControl, '/science/serial/auger',                lambda msg: self.actuator_control_callback(msg, AUGER_ACTUATOR_INDEX), 10)
        self.create_subscription(ScienceActuatorControl, '/science/serial/primary_cache_door',   lambda msg: self.actuator_control_callback(msg, PRIMARY_DOOR_ACTUATOR_INDEX), 10)
        self.create_subscription(ScienceActuatorControl, '/science/serial/secondary_cache_door', lambda msg: self.actuator_control_callback(msg, SECONDARY_DOOR_ACTUATOR_INDEX), 10)
        self.create_subscription(ScienceActuatorControl, '/science/serial/secondary_cache',      lambda msg: self.actuator_control_callback(msg, SECONDARY_CACHE_ACTUATOR_INDEX), 10)
        self.create_subscription(ScienceActuatorControl, '/science/serial/drill',                lambda msg: self.actuator_control_callback(msg, DRILL_ACTUATOR_INDEX), 10)
        self.create_subscription(Bool, '/science/serial/override', self.set_override_bit_callback, 10)
        self.create_subscription(Empty, '/science/serial/reset', self.establish_serial_connection, 10)
        self.create_subscription(Empty, '/science/emergency_stop', self.emergency_stop, 10)
        self.create_subscription(String, '/science/send_file', lambda msg: self.send_file_contents(msg.data), 10)
        self.create_subscription(Float32, '/science/calibrate_uv', lambda msg: self.uvsensor_calibrate(msg.data), 10)

        # Serial Communication Exchange
        self.sub_science_serial_tx_request = self.create_subscription(ScienceSerialTxPacket, '/science/serial/tx_request', self.perform_tx_request, 10)
        self.pub_science_serial_tx_notification = self.create_publisher(UInt8MultiArray, '/science/serial/tx_notification', 10)
        self.pub_science_serial_rx_notification = self.create_publisher(UInt8MultiArray, '/science/serial/rx_notification', 10)
        self.pub_science_serial_rx_packet = self.create_publisher(ScienceSerialRxPacket, '/science/serial/rx_packet', 10)

        # Timer to read the serial input
        self.create_timer(100e-3, self.read_serial) # 10 Hz

        # Data structure for reading packets
        self.read_queue = []

        # State Variable
        self.override_bit = False

    def establish_serial_connection(self, msg: Empty = None):
        for path in ["/dev/rover/scienceArduinoNano", "/dev/ttyUSB0"]:
            try:
                self.connect_serial(path)
                self.get_logger().info(f"Connected to {path}")
                return
            except Exception as e:
                self.get_logger().error(f"Error: Could not connect to {path}")
                self.get_logger().error(str(e))
        # Abort if no connection is made
        if serial.arduino is None:
            self.get_logger().error("Error: No serial connection established")
            self.get_logger().error("Exiting...")
            rclpy.shutdown()
            exit(0)

    def connect_serial(self, device_path):
        self.get_logger().info("Resetting serial connection...")
        if self.arduino is not None:
            self.arduino.close()
        self.arduino = serial.Serial(device_path, BAUD_RATE)
        self.get_logger().info("Serial port initialized")

    # Callbacks for Subscribers

    def actuator_control_callback(self, msg: ScienceActuatorControl, sensor_index):
        self.perform_tx_request(SMFL_Builder.get_tx_update_actuator_control(sensor_index, msg.control))

    def set_override_bit_callback(self, msg: Bool):
        self.override_bit = msg.data

    def emergency_stop(self, msg: Empty):
        # Emergency stop for all actuators
        prev_override_bit = self.override_bit
        self.override_bit = True
        self.perform_tx_request(SMFL_Builder.get_tx_abort_routine())
        for index in range(0, TOTAL_ACTUATORS):
            if index != DRILL_ACTUATOR_INDEX:
                #print(index, type(index))
                self.perform_tx_request(SMFL_Builder.get_tx_clear_positional_controller(index))
                self.perform_tx_request(SMFL_Builder.get_tx_clear_speed_controller(index))
            self.perform_tx_request(SMFL_Builder.get_tx_free_actuator(index))
            self.perform_tx_request(SMFL_Builder.get_tx_update_actuator_control(index, 0))
        self.override_bit = prev_override_bit
        self.get_logger().warning("Emergency stop activated, all actuators disabled")

    def uvsensor_calibrate(self, uvindex: float):
        self.get_logger().warn(f"Calibrating UV sensor with index {uvindex}")
        self.perform_tx_request(SMFL_Builder.get_tx_calibrate_uv_index(uvindex))

    def send_file_contents(self, file_path):
        # Read the raw bytes of the file
        try:
            with open(file_path, 'rb') as file:
                contents = file.read()

            # Send the raw bytes to the Arduino
            self.write_serial(contents)

            # Publish the raw bytes to the ROS topic
            msg = UInt8MultiArray()
            msg.data = list(contents)
            self.pub_science_serial_tx_notification.publish(msg)
        except:
            self.get_logger().error(f"Failed to read file {file_path}. Please ensure the file exists and is accessible to the science_serial_interface node.")

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
            self.write_serial(ScienceSerialInterface.author_packet(msg, self.override_bit))

    def write_serial(self, packet):
        self.arduino.write(struct.pack('B' * len(packet), *packet))
        self.publish_serial_tx_notification(packet)

    @staticmethod
    def author_packet(tx: ScienceSerialTxPacket, override=False):
        byte_array = [tx.command_word, len(tx.operands)] + list(tx.operands)
        if len(byte_array) > MAXIMUM_PACKET_SIZE:
            raise ValueError(f"Provided command packet is of size {len(byte_array)}, maximum is {MAXIMUM_PACKET_SIZE}")
        byte_array[0] |= 0b01000000 if override else 0b00000000  # Set the Override Bit
        byte_array.insert(0, COMMAND_PACKET_HEADER)
        byte_array.append(COMMAND_PACKET_FOOTER)
        return byte_array

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
            self.get_logger().info(f'Received Response Packet [len={len(error_message)}]:({error_code}) {bytes(error_message).decode()}')
        except:
            self.get_logger().info(f'Received Response Packet with invalid ascii:({error_code}) {",".join([ hex(x) for x in error_message])}')

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