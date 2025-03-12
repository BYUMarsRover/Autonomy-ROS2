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

        self.curr_wheel_msg = None
        self.curr_elev_msg = None
        self.curr_heart_msg = None
        self.last_wheel_msg_time = time.time()
        self.last_elev_msg_time = time.time()

        # SUBSCRIBERS
        self.create_subscription(IWCMotors, '/IWC_motorControl', self.send_wheel, 1)
        self.create_subscription(Elevator, '/elevator', self.send_elevator, 1)

        # PUBLISHERS
        self.pub_Debug = self.create_publisher(String, '/ArduinoDebug', 25)

        # self.serial_queue = queue.Queue()
        self.lock = threading.Lock()
        # Connect to Arduino
        self.disconnected = True
        self.handshake = True   # If a Serial object is good to communicate on the port
        self.connect()

        time.sleep(2.0)

        # Start writer thread
        self.writer_thread = threading.Thread(target=self.serial_writer_loop, daemon=True)
        self.writer_thread.start()

        self.create_timer(1.0, self.message_check) 
        self.get_logger().info("MegaMiddle Man started")
        self.write_debug("MegaMiddle Man started")


    def connect(self):
        failure_count = 0
        arduino_port = "/dev/rover/onBoardMega"
        while failure_count < 10:
            try:
                self.ser = serial.Serial(arduino_port, 115200, timeout=0.3, write_timeout=1.0, dsrdtr=True)
                self.write_debug("Orin: Connection handshake.")
                self.disconnected = False
                
                try:
                    self.ser.reset_input_buffer()
                    self.ser.reset_output_buffer()
                except Exception as buffer_exception:
                    self.write_debug(f"Orin: Could not reset buffers on connection: {buffer_exception}")
                
                return
            except serial.SerialException as e:
                self.get_logger().warn(f"Could not open serial port {arduino_port}: {e}")
                failure_count += 1
            
            time.sleep(0.1)

        # If this is reached
        self.get_logger().warn(f"Could not open serial port {arduino_port} after {failure_count} attempts.")
        self.write_debug("Orin: Could not establish connection to Arduino.")
        self.disconnect()
            

    def disconnect(self):
        self.disconnected = True
        # self.handshake = False
        #TODO try excepth here
        self.ser.close()
    
    
    # def serial_queue_write(self, msg):
    #     if not self.disconnected:
    #         try:
    #             self.serial_queue.put(msg)serial_queue_write
    #                 # while not self.serial_queue.empty():
    #                 #     message = self.serial_queue.get()
    #                     # self.ser.write(message.encode('ascii'))

    #         except serial.SerialException as e:
    #             self.write_debug("Orin: Failed to add message to queue.")
    #     else:
    #         self.write_debug("Orin: Not connected to Arduino; ignoring message.")

    def serial_writer_loop(self):
        while not self.disconnected: #TODO See if ROS2 has a if node is running
            with self.lock:
                # Ensure connection
                if not self.ser.is_open:
                    self.get_logger().warn("Orin: Arduino port is not open at time of writing!")
                    self.write_debug("Orin: Arduino port is not open at time of writing!")
                    return
                
                # # Ensure queue is reasonable
                # if self.serial_queue.qsize() > 5:
                #     self.get_logger().warn(f"Orin -> Mega queue is getting too long, culling data.")
                #     self.write_debug("Orin: Write queue is getting too long! Culling stale data.")
                #     self.serial_queue.queue.clear()
                #     return
                messages = []
                if self.curr_elev_msg is not None:
                    messages.append(self.curr_elev_msg)
                if self.curr_heart_msg is not None:
                    messages.append(self.curr_heart_msg)
                if self.curr_wheel_msg is not None:
                    messages.append(self.curr_wheel_msg)

                # Send message
                for msg in messages:
                    try:
                        self.ser.write((msg).encode('utf-8'))
                        self.write_debug("Orin: Writing to Arduino: " + msg)
                    except (serial.SerialTimeoutException) as e:
                        self.get_logger().warn(f"Failed to write to serial due to timeout: {e}")
                        self.write_debug("Orin: Failed to send message to Arduino in time.")
                        # 2024-25 Experiments revealed these can help with reliability
                        time.sleep(2) 
                        # self.handshake = False
                    except (serial.SerialException) as e:
                        self.get_logger().warn(f"Failed to write to serial due to exception: {e}")
                        self.write_debug("Orin: Error when trying to send message to Arduino.")
                        self.disconnect() # Not necessarily disconnected, but act like we are
                    except:
                        self.get_logger().warn(f"Failed to write to serial due to exception: UNKOWN EXCEPTION")
                        self.write_debug("Orin: Error when trying to send message to Arduino.")


                    # Give arduino breathing time
                    time.sleep(0.01) # [s], delay between writes, only blocks write thread not main thread.
                                    # Lowering this value can improve input latency, but lowering it too much
                                    # risks overwhelming the Arduino and causing a I/O timeout -- resulting 
                                    # in a few seconds of disconnect (not worth shaving a few ms of each volley)
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
        wheel_msg = "$WHEEL," + ",".join(str(int(param)) for param in motor_params) + "*"
        self.write_debug(f"Orin: Adding wheel message. {wheel_msg}")
        self.curr_wheel_msg = wheel_msg
        self.last_wheel_msg_time = time.time()

    def send_elevator(self, msg):
        eleva_params = [msg.elevator_speed, msg.elevator_direction]
        eleva_msg = "$ELEVA," + ",".join(str(int(param)) for param in eleva_params) + "*"
        self.write_debug(f"Orin: Adding elevator message to Queue. {eleva_msg}")
        self.curr_elev_msg = eleva_msg
        self.last_elev_msg_time = time.time()

    def write_debug(self, msg):
        message = String()
        message.data = msg
        self.pub_Debug.publish(message)

    def message_check(self):
        # Check if new data has come in the last 1 second
        timeout = 0.5
        current_time = time.time()
        if (current_time - self.last_wheel_msg_time > timeout) and (self.curr_wheel_msg is not None):
            # If no messages have come in the last second, clear the current messages
            self.curr_wheel_msg = None
            self.write_debug("Orin: No new wheel msg received; clearing messages.")

        if (current_time - self.last_elev_msg_time > timeout) and (self.curr_elev_msg is not None):
            # If no messages have come in the last second, clear the current messages
            self.curr_elev_msg = None
            self.write_debug("Orin: No new elev msg received; clearing messages.")


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
