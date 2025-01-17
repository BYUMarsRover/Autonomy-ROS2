#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, UInt16MultiArray
from rover_msgs.msg import IWCMotors, Elevator, HeartbeatStatusRover #, FPVServo
import serial
import time
import queue
import threading

class MegaMiddleman(Node):
    def __init__(self):
        super().__init__('mega_middleman')

        # SUBSCRIBERS
        self.create_subscription(IWCMotors, '/IWC_motorControl', self.send_wheel, 1)
        self.create_subscription(Elevator, '/elevator', self.send_elevator, 1)
        self.create_subscription(Bool, '/arm_clicker', self.send_clicker, 1)
        self.create_subscription(Bool, '/arm_laser', self.send_laser, 1)
        # self.create_subscription(FPVServo, '/fpv_servo', self.send_fpvsv, 1)
        self.create_subscription(HeartbeatStatusRover, '/heartbeat_status_rover', self.send_heart, 1)

        # PUBLISHERS
        self.pub_IR = self.create_publisher(UInt16MultiArray, '/IR', 1)
        self.pub_Debug = self.create_publisher(String, '/ArduinoDebug', 25)

        self.serial_queue = queue.Queue()
        self.lock = threading.Lock()
        # Connect to Arduino
        self.disconnected = True
        self.connect()

        # Timer for relay_mega
        self.create_timer(0.1, self.relay_mega)  # 10 Hz

    def connect(self):
        failure_count = 0
        while failure_count < 10:
            try:
                arduino_port = "/dev/rover/onBoardMega"
                self.ser = serial.Serial(arduino_port, 115200, timeout=4.0)
                break
            except serial.SerialException as e:
                self.get_logger().warn(f"Could not open serial port {arduino_port}: {e}")
                failure_count += 1
            time.sleep(1) #TODO
        if failure_count >= 10:
            self.get_logger().warn(f"Could not open serial port {arduino_port} after {failure_count} attempts.")
            self.write_debug("Orin: Could not establish connection to Arduino.")
            self.disconnected = True
        else:
            self.write_debug("Orin: Connection handshake.")
            self.disconnected = False

    def serial_write(self, msg):
        if not self.disconnected:
            try:
                with self.lock:
                    self.serial_queue.put(msg)
                    while not self.serial_queue.empty():
                        message = self.serial_queue.get()
                        self.ser.write(message.encode('ascii'))

            except serial.SerialException as e:
                self.write_debug("Orin: Failed to send message to Arduino.")
                self.disconnected = True # Not necessarily disconnected, but could be
        else:
            self.write_debug("Orin: Not connected to Arduino; ignoring message.")

    def send_wheel(self, msg):
        motor_params = [
            msg.left_front_speed, msg.left_front_dir,
            msg.left_middle_speed, msg.left_middle_dir,
            msg.left_rear_speed, msg.left_rear_dir,
            msg.right_front_speed, msg.right_front_dir,
            msg.right_middle_speed, msg.right_middle_dir,
            msg.right_rear_speed, msg.right_rear_dir
        ]
        wheel_msg = "$WHEEL," + ",".join(str(param) for param in motor_params) + "*"
        self.serial_write(wheel_msg)

    def send_elevator(self, msg):
        eleva_params = [msg.elevator_speed, msg.elevator_direction]
        eleva_msg = "$ELEVA," + ",".join(str(param) for param in eleva_params) + "*"
        self.write_debug("Orin: Sending elevator message to Arduino.")
        self.serial_write(eleva_msg)

    def send_clicker(self, msg):
        click_value = 1 if msg.data else 0
        click_msg = f"$CLICK,{click_value}*"
        self.serial_write(click_msg)

    def send_laser(self, msg):
        laser_value = 1 if msg.data else 0
        laser_msg = f"$LASER,{laser_value}*"
        self.serial_write(laser_msg)

    # def send_fpvsv(self, msg):
    #     fpv_params = [msg.yaw, msg.pitch]
    #     fpvsv_msg = "$FPVSV," + ",".join(str(param) for param in fpv_params) + "*"
    #     self.serial_write(fpvsv_msg)

    def send_heart(self, msg):
        heart_msg = f"$HEART,{msg.elapsed_time}*"
        self.serial_write(heart_msg)

    def read_nmea(self):
        try:
            x = self.ser.read(1).decode('ascii').strip()
            if not x:
                self.write_debug("Orin: Nothing to read")
                return 0, ""
        except:
            self.write_debug("WARNING: Read failure")
            self.ser.reset_input_buffer()
            return -1, ""

        self.write_debug("Orin: Reading a new message from the Arduino starting with " + x)
        if x != '$':
            self.ser.reset_input_buffer()
            return -1, ""

        try:
            mega_msg = self.ser.read_until(b'*').decode('ascii').strip()
        except:
            self.write_debug("WARNING: Read failure")
            self.ser.reset_input_buffer()
            return

        if ',' in mega_msg:
            msg_parts = mega_msg.split(',', 1)
            tag, data = msg_parts[0], msg_parts[1][0:-2]
        else:
            tag, data = mega_msg, ""
        self.write_debug(f"Orin: Reading a message from the Arduino w/ tag: {tag}, and data: {data}")

        self.ser.reset_input_buffer()
        return tag, data

    def relay_mega(self):
        tag, msg = self.read_nmea()

        if tag == "IRLIG":
            try:
                sensor_data = UInt16MultiArray(data=[int(x) for x in msg.split(',')])
                self.pub_IR.publish(sensor_data)
            except ValueError:
                self.write_debug("Orin: Failed to parse IR data")
                self.get_logger().warn("Failed to parse IR data")
        elif tag == "DEBUG":
            self.pub_Debug.publish(String(data=msg))
        elif tag == -1:
            self.write_debug("Orin: Read corrupt or incomplete message.")

    def write_debug(self, msg):
        self.pub_Debug.publish(String(data=msg))

def main(args=None):
    rclpy.init(args=args)
    middleman = MegaMiddleman()
    try:
        rclpy.spin(middleman)
    except KeyboardInterrupt:
        pass
    finally:
        middleman.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
