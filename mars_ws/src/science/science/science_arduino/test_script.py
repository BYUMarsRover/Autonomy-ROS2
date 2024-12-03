#!/usr/bin/env python3
"""
Test Serial for Science Arduino

WARNING: This test control software is NOT up to date, and is meant as a reference.
Adjust to the latest serial packet structure and adjust the script contents before running.
"""

import time
import serial
import struct

class TestScienceSerial:

    def __init__(self):
        # 
        self.arduino = serial.Serial("/dev/ttyUSB0", 9600)
        
        # packet values
        self.current_auger_index = 0	# which auger, 0 (left) or 1 (right)
        self.linear_actuator_speed = 0 	# speed, -127 to 127
        self.auger_speed = 0			# speed, -127 to 127
        self.servo_position = 1			# 1 (closed), 0 (open)

        # constants
        self.PACKET_DELIMITER = 0x80

    # convert int8 to uint8
    def signed_to_unsigned(self, signed_value):
        if signed_value < 0:
            # compute the two's complement representation
            unsigned_value = (1 << 8) + signed_value
        else:
            unsigned_value = signed_value
        return unsigned_value

    # Callback function for when a science message gets published
    def write_serial(self):
        if self.arduino:
            message_bytes = [
                self.PACKET_DELIMITER,
                self.signed_to_unsigned(self.current_auger_index),
                self.signed_to_unsigned(self.linear_actuator_speed),
                self.signed_to_unsigned(self.auger_speed),
                self.signed_to_unsigned(self.servo_position)
            ]
            # write out the string to the arduino
            self.arduino.write(struct.pack('B' * len(message_bytes), *message_bytes))

    def read_serial(self) -> list[int]:
        # only attempt reads when there are available bytes on the Serial input buffer
        if (self.arduino.in_waiting > 0):
            # discard a line to reset to correct position
            self.arduino.readline()
            # make sure there's data??
            # get data until a '\n' character (blocking), decode it, strip off the \n, then split it by :
            data = self.arduino.readline()
            data = data.decode('utf-8').strip().split(':')
            data = [int(n) for n in data]
            return data
        else:
            return None

if __name__ == '__main__':

    test = TestScienceSerial()

    while not test.arduino:
        pass
    print("Science Serial Online\n")

    # print("# AUGER TESTS ####################\n")

    # test.current_auger_index = 1

    # print("# test down\n")
    # test.linear_actuator_speed = -127
    # test.write_serial()
    # time.sleep(7)
    # test.linear_actuator_speed = 0
    # test.write_serial()

    # print("# test up\n")
    # test.linear_actuator_speed = 127
    # test.write_serial()
    # time.sleep(5)
    # test.linear_actuator_speed = 0

    # print("# test drill forward\n")
    # test.auger_speed = 127
    # test.write_serial()
    # time.sleep(5)

    # print("# test drill reverse\n")
    # test.auger_speed = -127
    # test.write_serial()
    # time.sleep(5)
    # test.auger_speed = 0

    # print("# test servo open\n")
    # test.servo_position = 0
    # test.write_serial()
    # time.sleep(5)

    # print("# test servo close\n")
    # test.servo_position = 1
    # test.write_serial()
    # time.sleep(5)

    # print("# PROBE TESTS ####################\n")

    # test.current_auger_index = 0

    # print("# test down\n")
    # test.linear_actuator_speed = 127
    # test.write_serial()
    # time.sleep(0.5)
    # test.linear_actuator_speed = 0
    # test.write_serial()


    # print("# test up\n")
    # test.linear_actuator_speed = 127
    # test.write_serial()
    # time.sleep(5)
    # test.linear_actuator_speed = 0

    print("# test read\n")
    while(True):
        print(test.read_serial(), "\n")

    # test.current_auger_index = 0

    # Drill, Auger Down ######################################
    # print("# test drill forward\n")
    # test.auger_speed = 127
    # test.write_serial()


    # while(True):
    #     print("# test probe down\n")
    #     test.linear_actuator_speed = -127
    #     test.write_serial()
    #     time.sleep(0.5)
    #     print("# test probe up\n")
    #     test.linear_actuator_speed = 127
    #     test.write_serial()
    #     time.sleep(0.5)

    # while(True):
    #     print("# test servo open\n")
    #     test.servo_position = 0
    #     test.write_serial()
    #     time.sleep(5)
    #     print("# test servo close\n")
    #     test.servo_position = 1
    #     test.write_serial()
    #     time.sleep(5)


# Test Run #####################################################
    
    # print("# test servo open\n")
    # test.servo_position = 0
    # test.write_serial()
    # time.sleep(1)
    # test.auger_speed = 127
    # test.write_serial()
    # print("# test down\n")
    # test.linear_actuator_speed = -127
    # test.write_serial()
    # time.sleep(10)
    # print("# test servo close\n")
    # test.auger_speed = 0
    # test.write_serial()
    # test.servo_position = 1
    # test.write_serial()
    # time.sleep(2)
    # test.auger_speed = 127
    # test.write_serial()
    # time.sleep(10)

    # # Reverse Drill, Auger Up ###################################
    # print("# test up\n")
    # test.linear_actuator_speed = 127
    # test.write_serial()

    # print("# test drill reverse\n")
    # test.auger_speed = -127
    # test.write_serial()
    
    # time.sleep(10)

    # print("# test drill stop\n")
    # test.auger_speed = 0
    # test.write_serial()

#############################################################
    
    # time.sleep(0.5)
    # test.linear_actuator_speed = 0
    # test.write_serial()
    
    # print("# test drill stop\n")
    # test.auger_speed = 0
    # test.write_serial()

    # print("# test servo open\n")
    # test.servo_position = 0
    # test.write_serial()
    # time.sleep(1)

    if test.arduino:
        test.arduino.close()
        test.arduino = None