import rclpy
from rclpy.node import Node
# import serial
import os
# import beepy as
import time
from rover_msgs.msg import RawBattery
# from rover_msgs.msg import RoverState, Battery
#from rover_msgs.msg import Battery

# Battery definitions and mapping.

# PREAMBLE_BATTERY = 2 #in binary

BATTERY_OFF = 10
BATTERY_LOW_VOLTAGE_LEVEL = 40
BATTERY_FULL_CHARGE_LEVEL = 48


THEORETICAL_STEP_DOWN_LOW_VOLTAGE = 3.03
THEORETICAL_STEP_DOWN_HIGH_VOLTAGE = 3.68

ADC_GROUND_MAPPING = 104
ADC_5V_MAPPING = 833


ADC_VOLTAGE_LOWER_MAPPING = (THEORETICAL_STEP_DOWN_HIGH_VOLTAGE - THEORETICAL_STEP_DOWN_LOW_VOLTAGE) * \
    (ADC_5V_MAPPING - ADC_GROUND_MAPPING) + ADC_GROUND_MAPPING
ADC_VOLTAGE_HIGHER_MAPPING = (THEORETICAL_STEP_DOWN_HIGH_VOLTAGE - THEORETICAL_STEP_DOWN_LOW_VOLTAGE) * (
    THEORETICAL_STEP_DOWN_HIGH_VOLTAGE - THEORETICAL_STEP_DOWN_LOW_VOLTAGE) / (ADC_5V_MAPPING - ADC_GROUND_MAPPING) + ADC_GROUND_MAPPING

print("Battery Publishing Init, using upper adc value",
      ADC_VOLTAGE_HIGHER_MAPPING, "using lower adc value", ADC_VOLTAGE_LOWER_MAPPING)
#rospy.init_node('battery_info', anonymous=True)
# battery_info = rospy.Publisher("battery_info", Battery, queue_size = 1)
print("Battery Publishing Online")

# battery = Battery()

class BatteryInfo(Node):
    def __init__(self):
        super().__init__('battery_info')
        self.subscription = self.create_subscription(
            RawBattery,
            'raw_battery_info',
            self.checkBattery,
            1  # QoS profile depth
        )
        self.get_logger().info("Battery Publishing Online")

    def checkBattery(self, data):

        # Data array padded on both ends with 0's to prevent data from being out of order.
        # Preamble is at 1, low byte is at 2, high byte is at 3. 0's at indexes 0 and 4.

        rawadc = data.voltage

        print(rawadc)
        # if (preamble == PREAMBLE_BATTERY):
        # Voltage was sent over as voltage*10 to avoid using decimals (could be sent over as two chars, rather than floating point),
        # so divide by ten to get real voltage number
        voltage = (rawadc - ADC_VOLTAGE_LOWER_MAPPING) / ((ADC_VOLTAGE_HIGHER_MAPPING - ADC_VOLTAGE_LOWER_MAPPING) /
                                                        (BATTERY_FULL_CHARGE_LEVEL - BATTERY_LOW_VOLTAGE_LEVEL)) + BATTERY_LOW_VOLTAGE_LEVEL
        percentage = round((voltage-BATTERY_LOW_VOLTAGE_LEVEL) /
                        (BATTERY_FULL_CHARGE_LEVEL-BATTERY_LOW_VOLTAGE_LEVEL)*100, 1)

        status_string = f"Battery status: {voltage}V ({percentage}%)"
        self.get_logger().info(status_string)

        if BATTERY_OFF < voltage < BATTERY_LOW_VOLTAGE_LEVEL:
            self.get_logger().warn("Battery low, initiating shutdown.")
            print('\a')  # Terminal bell sound
            time.sleep(1)
            # Uncomment the shutdown line below if needed
            # os.system("shutdown now")

def main(args=None):
    rclpy.init(args=args)
    battery_info = BatteryInfo()
    rclpy.spin(battery_info)
    battery_info.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
