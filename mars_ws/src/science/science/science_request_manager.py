"""
PURPOSE: Acts as an abstraction layer between requests for data from the science module and the transmission of actual Tx packets.

SUBSCRIBED TO:
- /science_sensor_request (std_msgs/Empty): Requests for analog sensor data.
- /science_serial_rx_packet (rover_msgs/ScienceSerialRxPacket): Responses from the science module.

PUBLISHES TO:
- /science_sensor_values (rover_msgs/ScienceSensorValues): Publishes processed sensor data.
- /science_serial_tx_request (rover_msgs/ScienceSerialTxPacket): Sends requests to the science module.

FUNCTIONALITY:
- Handles requests for raw and calibrated analog sensor data.
- Manages communication with the science module via Tx and Rx packets.
- Processes and forwards sensor data to appropriate topics.
"""

import rclpy
from rclpy.node import Node
from rover_msgs.msg import ScienceSerialRxPacket, ScienceSerialTxPacket, ScienceSensorValues, ScienceSpectroData, ScienceUvData
from std_msgs.msg import Empty, UInt8MultiArray, Bool
from science.function_mapping.function_map import ScienceModuleFunctionList as SMFL
import struct

TEMP_SENSOR_INDEX = 0
HUM_SENSOR_INDEX = 1

class ScienceRequestManager(Node):
    """Bridge Science Requests and Serial Interface"""

    def __init__(self):
        super().__init__('science_request_manager')

        # Subscriptions to various request channels
        self.sub_get_uv = self.create_subscription(Empty, '/science_uv_request', self.uvsensor_request, 10)
        self.sub_get_analog_sensors = self.create_subscription(Bool, '/science_sensor_request', self.sensor_request, 10)
        self.sub_get_spectrograph_data = self.create_subscription(Empty, '/science_spectro_request', self.spectrograph_request, 10)

        # Publisher to various response channels
        self.pub_analog_sensors = self.create_publisher(ScienceSensorValues, '/science_sensor_values', 10)
        self.pub_spectrograph = self.create_publisher(ScienceSpectroData, '/science_spectro_data', 10)
        self.pub_uv_sensor = self.create_publisher(ScienceUvData, '/science_uv_data', 10)

        # Publisher to the request TX Channel
        self.pub_tx = self.create_publisher(ScienceSerialTxPacket, '/science_serial_tx_request', 10)

        # Subscriber to the response RX Channel
        self.sub_rx = self.create_subscription(ScienceSerialRxPacket, '/science_serial_rx_packet', self.parse_rx, 10)

        # Observers to parse the RX
        self.raw_sensor_observer = RawAnalogSensorValues(SMFL.get_function_by_function_name('get_analog_sensor_raw'), self.pub_tx, self.pub_analog_sensors)
        self.calibrated_sensor_observer = CalibratedAnalogSensorValues(SMFL.get_function_by_function_name('get_analog_sensor_calibrated'), self.pub_tx, self.pub_analog_sensors)
        self.spectrograph_observer = Spectrograph(SMFL.get_function_by_function_name('return_spectrograph_data'), self.pub_tx, self.pub_spectrograph)
        self.uvsensor_observer = UVSensor(SMFL.get_function_by_function_name('return_ltr_data'), self.pub_tx, self.pub_uv_sensor)
        self.observers = [self.raw_sensor_observer, self.calibrated_sensor_observer, self.spectrograph_observer, self.uvsensor_observer]

    def sensor_request(self, msg: Bool):
        if msg.data == True:
            self.calibrated_sensor_observer.request()
        else:
            self.raw_sensor_observer.request()

    def spectrograph_request(self, msg: Empty):
        self.spectrograph_observer.request(self)

    def uvsensor_request(self, msg: Empty):
        self.uvsensor_observer.request(self)

    def parse_rx(self, rx: ScienceSerialRxPacket):
        # Ignore anythign with an error code
        if rx.error_code != 0:
            return
        # Iterate over observers
        for observer in self.observers:
            # If the rx packet has the same command word are this observer
            if SMFL.get_command_word(observer.func_def) == rx.echo[1] & 0b10011111:
                # self.get_logger().info(f"Got callback for function {observer.func_def['function_name']}")
                observer.receive(rx)
                return


class RxObserver:
    def __init__(self, func_def, pub_tx, pub_forward):
        # This func_def should match the function that requests the data
        self.func_def = func_def
        self.pub_tx = pub_tx
        self.pub_forward = pub_forward
        self.observing = False

    def isObserving(self):
        return self.observing
    
    def receive(self, rx: ScienceSerialRxPacket):
        pass

class AnalogSensorValues(RxObserver):
    def __init__(self, func_def, pub_tx, pub_forward, tx_packet_builder):
        # This func_def should match the function that requests the data 
        super().__init__(func_def, pub_tx, pub_forward)
        self.tx_packet_builder = tx_packet_builder
        self.reset()

    def reset(self):
        self.observing = False
        self.temperature = None
        self.humidity = None

    def request(self):
        if self.observing:
            return
        else:
            self.pub_tx.publish( self.tx_packet_builder(TEMP_SENSOR_INDEX) )
            self.pub_tx.publish( self.tx_packet_builder(HUM_SENSOR_INDEX) )
            self.observing = True

    def receive(self, rx):
        data = SMFL.get_return_data(self.func_def, rx)
        if rx.echo[3] == TEMP_SENSOR_INDEX: # Sensor Index
            self.receiveTemp(data)
        else:
            self.receiveHumidity(data)

    def receiveTemp(self, data):
        self.temperature = data
        self.check_complete()

    def receiveHumidity(self, data):
        self.humidity = data
        self.check_complete()

    def check_complete(self):
        if self.temperature is not None and self.humidity is not None:
            self.pub_forward.publish(
                ScienceSensorValues(
                    temperature = int(self.temperature),
                    moisture = int(self.humidity)
                )
            )
            self.reset()

class RawAnalogSensorValues(AnalogSensorValues):
    def __init__(self, func_def, pub_tx, pub_response):
        super().__init__(func_def, pub_tx, pub_response, SMFL.get_tx_get_analog_sensor_raw)

class CalibratedAnalogSensorValues(AnalogSensorValues):
    def __init__(self, func_def, pub_tx, pub_response):
        super().__init__(func_def, pub_tx, pub_response, SMFL.get_tx_get_analog_sensor_calibrated)

class Spectrograph(RxObserver):
    def __init__(self, func_def, pub_tx, pub_forward):
        super().__init__(func_def, pub_tx, pub_forward)
        self.timer = None
        self.reset()

    def reset(self):
        self.observing = False
        self.channels = None
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None

    def request(self, node):
        if self.observing:
            return
        else:
            num_samples = 1
            sample_duration_ms = 1000
            bulb_on = True
            self.pub_tx.publish( SMFL.get_tx_get_sample_spectrograph(num_samples, sample_duration_ms, bulb_on) )
            self.observing = True
            self.timer = node.create_timer(1.0, self.check_data_is_ready) # Check every second

    def check_data_is_ready(self):
        # Request data from the Science Module
        self.pub_tx.publish( SMFL.get_tx_return_spectrograph_data() )

    def receive(self, rx):
        self.pub_forward.publish(
                ScienceSpectroData(
                    values = SMFL.get_return_data(self.func_def, rx)
                )
            )
        self.reset()

class UVSensor(RxObserver):
    def __init__(self, func_def, pub_tx, pub_forward):
        super().__init__(func_def, pub_tx, pub_forward)
        self.timer = None
        self.reset()

    def reset(self):
        self.observing = False
        self.channels = None
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None

    def request(self, node):
        if self.observing:
            return
        else:
            self.pub_tx.publish( SMFL.get_tx_sample_ltr() )
            self.observing = True
            self.timer = node.create_timer(0.1, self.check_data_is_ready) # Check every second

    def check_data_is_ready(self):
        # Request data from the Science Module
        self.pub_tx.publish( SMFL.get_tx_return_ltr_data() )

    def receive(self, rx):
        data = SMFL.get_return_data(self.func_def, rx)
        self.pub_forward.publish(
                ScienceUvData(
                    als = data[1], # Second Float
                    uv = data[0] # First Float
                )
            )
        self.reset()

def main(args=None):
    rclpy.init(args=args)
    science_request_manager = ScienceRequestManager()

    science_request_manager.get_logger().info('Science Request Manager Online')
    rclpy.spin(science_request_manager)
    science_request_manager.destroy_node()
    rclpy.shutdown

if __name__ == '__main__':
    main()