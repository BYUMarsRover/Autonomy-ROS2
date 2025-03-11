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
        self.handshake = False   # If a Serial object is good to communicate on the port
        self.connect()

        time.sleep(7.0)

        # Start writer thread
        self.writer_thread = threading.Thread(target=self.serial_writer_loop, daemon=True)
        self.writer_thread.start()

        # Timer for relay_mega
        self.create_timer(0.1, self.loop)  # 100 Hz
        self.get_logger().info("MegaMiddle Man started")

    def connect(self):
        failure_count = 0
        while failure_count < 10:
            try:
                arduino_port = "/dev/rover/onBoardMega"
                self.ser = serial.Serial(arduino_port, 115200, timeout=0.3, write_timeout=1.0, dsrdtr=True)
                break
            except serial.SerialException as e:
                self.get_logger().warn(f"Could not open serial port {arduino_port}: {e}")
                failure_count += 1
            time.sleep(0.1) #TODO
        if failure_count >= 10:
            self.get_logger().warn(f"Could not open serial port {arduino_port} after {failure_count} attempts.")
            self.write_debug("Orin: Could not establish connection to Arduino.")
            self.disconnected = True
            self.disconnect()
        else:
            self.write_debug("Orin: Connection handshake.")
            self.disconnected = False
            try:
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
            except:
                self.write_debug("Orin: Could not reset buffers on connection.")

    def disconnect(self):
        self.disconnected = True
        self.handshake = False
        self.ser.close()
    
    

    def serial_write(self, msg):
        if not self.disconnected:
            try:
                # Ensure queue is reasonable
                if self.serial_queue.qsize() > 5:
                    self.get_logger().warn(f"Orin -> Mega queue is getting too long, culling data.")
                    self.write_debug("Orin: Write queue is getting too long! Culling stale data.")
                    with self.lock:
                        self.serial_queue.queue.clear()
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

    def serial_writer_loop(self):
        while True: #TODO See if ROS2 has a if node is running
            if not self.handshake:
                self.write_debug("Orin: Waiting for Arduino handshake")
                time.sleep(1)
                continue
            msg = self.serial_queue.get()  # Blocking until a message is available
            if not self.disconnected:
                with self.lock:
                    # Ensure connection
                    if not self.ser.is_open:
                        self.get_logger().warn("Orin: Arduino port is not open at time of writing!")
                        self.write_debug("Orin: Arduino port is not open at time of writing!")
                        return
                    
                    # Ensure queue is reasonable
                    if self.serial_queue.qsize() > 20:
                        self.get_logger().warn(f"Orin -> Mega queue is getting too long, culling data.")
                        self.write_debug("Orin: Write queue is getting too long! Culling stale data.")
                        self.serial_queue.queue.clear()
                        return

                    # Send message
                    try:
                        self.ser.write((msg).encode('utf-8'))
                        self.write_debug("Orin: Thread output to Arduino: " + msg)
                    except (serial.SerialTimeoutException) as e:
                        self.get_logger().warn(f"Failed to write to serial due to timeout: {e}")
                        self.write_debug("Orin: Failed to send message to Arduino in time.")
                        # 2024-25 Experiments revealed these can help with reliability
                        time.sleep(2) 
                        self.handshake = False
                    except (serial.SerialException) as e:
                        self.get_logger().warn(f"Failed to write to serial due to exception: {e}")
                        self.write_debug("Orin: Error when trying to send message to Arduino.")
                        self.disconnect() # Not necessarily disconnected, but act like we are


                    # Give arduino breathing time
                    time.sleep(0.01) # [s], delay between writes, only blocks write thread not main thread.
                                     # Lowering this value can improve input latency, but lowering it too much
                                     # risks overwhelming the Arduino and causing a I/O timeout -- resulting 
                                     # in a few seconds of disconnect (not worth shaving a few ms of each volley)
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
        wheel_msg = "$WHEEL," + ",".join(str(int(param)) for param in motor_params) + "*"
        # self.write_debug(wheel_msg)
        self.serial_write(wheel_msg)

    def send_elevator(self, msg):
        eleva_params = [msg.elevator_speed, msg.elevator_direction]
        eleva_msg = "$ELEVA," + ",".join(str(int(param)) for param in eleva_params) + "*"
        # self.write_debug(f"Orin: Sending elevator message to Arduino. {eleva_msg}")
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
        # Watch for start of new message
        try:
            # Check if serial is open
            if not self.ser.is_open:
                self.get_logger().warn("Serial port is not open.")
                self.write_debug("WARNING: Serial port is not open")
                return -1, ""

            # Try reading one byte and decoding it
            x = self.ser.read(1).decode('ascii').strip()

            # Check if data was read
            if not x:
                self.get_logger().info("No data read from serial port.")
                # self.write_debug("Orin: Nothing to read")  # Optional: uncomment if you want this log
                return 0, ""

        except serial.SerialException as e:
            # Handle SerialException (e.g., if the port is not available)
            self.get_logger().error(f"SerialException: Failed to read from serial port: {e}")
            self.write_debug(f"ERROR: SerialException: {e}")

            # Try to reset the input buffer and disconnect
            try:
                self.ser.reset_input_buffer()  # Reset the buffer
                self.disconnect()  # Disconnect
                self.get_logger().info("Serial port input buffer reset and disconnected.")
            except Exception as reset_e:
                self.get_logger().warn(f"Could not flush input buffer: {reset_e}")
                self.write_debug(f"WARNING: Could not flush input buffer: {reset_e}")

            return -1, ""

        except Exception as e:
            # Catch all other unexpected errors
            self.get_logger().error(f"Unexpected error: {e}")
            self.write_debug(f"ERROR: Unexpected error: {e}")
            return -1, ""
        
        # Debug
        # self.write_debug("Orin: Reading a new message from the Arduino starting with " + x)
        # Verify message is in NMEA format
        if x != '$':
            return -1, ""
        
        # Read the rest of the sentence until '*'
        try:
            mega_msg = self.ser.read_until(b'*').decode('ascii').strip()
        except:
            self.get_logger().warn(f"Failed to read from serial - Second")
            self.write_debug("WARNING: Read failure - Second")
            try:
                self.ser.reset_input_buffer()
            except:
                self.write_debug("WARNING: Could not flush input buffer")
            return -1, ""
        
        # Ensure message does not contain encased messages
        if '$' in mega_msg:
            self.write_debug("WARNING: Nested messages: " + mega_msg)
            return -1, ""

        # Split message tag from data
        if ',' in mega_msg: 
            msg_parts = mega_msg.split(',', 1)
            tag, data = msg_parts[0], msg_parts[1][0:-2] # Strip off trailing ,*
        else:
            tag, data = mega_msg, ""
        # Debug
        self.write_debug("Orin: Reading a message from the Arduino w/ tag: " + tag + ", and data: " + data)

        return tag, data
    
    def relay_mega(self):
        # Read message
        ###################
        tag, msg = self.read_nmea()

        # Interpret message
        ###################
        if(tag == "IRLIG"):
            try:
                sensorData = UInt16MultiArray(data=[int(x) for x in msg.split(',')])
                self.pub_IR.publish(sensorData)
            except ValueError:
                self.write_debug("Orin: Failed to parse IR data")
                self.get_logger().warn("Failed to parse IR data")
        
        elif(tag == "DEBUG"):
            self.write_debug(msg)

        elif(tag == "HANDS"):
            self.handshake = True
            self.serial_write("$HANDS,*")
            self.write_debug("Orin: Recieved Arduino connection handshake.")
        
        # Debug
        elif(tag == -1):
            self.write_debug("Orin: Read corrupt or incomplete message.  (This is expected after a read error)")

    def write_debug(self, msg):
        message = String()
        message.data = msg
        self.pub_Debug.publish(message)

    def loop(self):
        if self.disconnected:
            self.connect()
        else:
            self.relay_mega()

    def write_nonsense(self):
    	self.serial_write("$ELEVA,0,0,*")

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
