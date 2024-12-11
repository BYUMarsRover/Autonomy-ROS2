import rclpy
from rclpy.node import Node
import serial
import os
import time
from rover_msgs.msg import Battery  # Update with the actual message type if different

# Battery definitions and mapping
BATTERY_OFF = 10
BATTERY_LOW_VOLTAGE_LEVEL = 40
BATTERY_FULL_CHARGE_LEVEL = 48

class BatteryInfoNode(Node):
    def __init__(self):
        super().__init__('battery_info')
        
        # Publisher setup for Battery info
        self.battery_publisher = self.create_publisher(Battery, 'battery_info', 10)
        
        # Serial port initialization
        try:
            self.serial_port = serial.Serial("/dev/cu.usbserial-AB0L9N2P", 9600)
            self.get_logger().info("Serial port initialized")
        except Exception as e:
            self.get_logger().error("Error: peripheralsBoard not yet ready")
            self.get_logger().error(str(e))
            rclpy.shutdown()
            exit(0)

        # Run the checkBattery method repeatedly
        self.check_battery_timer = self.create_timer(1.0, self.check_battery)

    def check_battery(self):
        # Flush serial data buffers to ensure clean start
        self.serial_port.flushInput()

        if self.serial_port.in_waiting:
            # Reading data from the serial port
            data = self.serial_port.read(5)
            voltage = data[0]  # Read the voltage byte
            preamble = ord(chr(data[1]))  # Convert byte to integer
            
            # Calculate battery percentage
            percentage = round((voltage - BATTERY_LOW_VOLTAGE_LEVEL) /
                               (BATTERY_FULL_CHARGE_LEVEL - BATTERY_LOW_VOLTAGE_LEVEL) * 100, 1)

            # Create and publish Battery message
            battery_msg = Battery()
            battery_msg.voltage = voltage
            battery_msg.percentage = percentage
            self.battery_publisher.publish(battery_msg)
            
            # Log battery status
            status_string = f"Battery status: {voltage}V ({percentage}%)"
            self.get_logger().info(status_string)

            # Handle low battery scenario
            if BATTERY_OFF < voltage < BATTERY_LOW_VOLTAGE_LEVEL:
                self.get_logger().warn("Battery low, initiating shutdown.")
                print('\a')  # Terminal bell sound
                # os.system("shutdown now")  # Uncomment to trigger shutdown if needed
                time.sleep(1)

            # Flush the input buffer to remove any old data
            self.serial_port.flushInput()

def main(args=None):
    rclpy.init(args=args)
    battery_info_node = BatteryInfoNode()
    rclpy.spin(battery_info_node)

    # Clean up and shutdown
    battery_info_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()