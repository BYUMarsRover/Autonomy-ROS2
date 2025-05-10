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


# Note to future devs
# This file is a prime target for refactoring. It is a bit of a mess and could be made much cleaner.
# The Observer scheme is very clean, but it works for now.

import rclpy
from rclpy.node import Node
from rover_msgs.msg import ScienceSerialRxPacket, ScienceSerialTxPacket, ScienceSensorValues, ScienceSpectroData, ScienceUvData, ScienceActuatorState
from std_msgs.msg import Empty, UInt8MultiArray, Bool, UInt8
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
        self.sub_get_actuator_state = self.create_subscription(UInt8, '/science_get_actuator_state', self.actuator_request, 10)

        # Publisher to various response channels
        self.pub_analog_sensors = self.create_publisher(ScienceSensorValues, '/science_sensor_values', 10)
        self.pub_spectrograph = self.create_publisher(ScienceSpectroData, '/science_spectro_data', 10)
        self.pub_uv_sensor = self.create_publisher(ScienceUvData, '/science_uv_data', 10)
        self.pub_actuator_state = self.create_publisher(ScienceActuatorState, '/science_actuator_state_update', 10)

        # Publisher to the request TX Channel
        self.pub_tx = self.create_publisher(ScienceSerialTxPacket, '/science_serial_tx_request', 10)

        # Subscriber to the response RX Channel
        self.sub_rx = self.create_subscription(ScienceSerialRxPacket, '/science_serial_rx_packet', self.parse_rx, 10)

        # Observers to parse the RX
        self.raw_sensor_observer = RawAnalogSensorValues(self.pub_tx, self.pub_analog_sensors)
        self.calibrated_sensor_observer = CalibratedAnalogSensorValues(self.pub_tx, self.pub_analog_sensors)
        self.spectrograph_observer = Spectrograph(self.pub_tx, self.pub_spectrograph)
        self.uvsensor_observer = UVSensor(self.pub_tx, self.pub_uv_sensor)
        self.observers = [self.raw_sensor_observer, self.calibrated_sensor_observer, self.spectrograph_observer, self.uvsensor_observer]

        # Complex Observer Types
        self.actuator_state_manager = ActuatorStateManager(self.pub_tx, self.pub_actuator_state, self.observers)

    def sensor_request(self, msg: Bool):
        if msg.data == True:
            self.calibrated_sensor_observer.request()
        else:
            self.raw_sensor_observer.request()

    def spectrograph_request(self, msg: Empty):
        self.spectrograph_observer.request(self)

    def uvsensor_request(self, msg: Empty):
        self.uvsensor_observer.request(self)

    def actuator_request(self, msg: UInt8):
        self.actuator_state_manager.request(msg.data)

    def parse_rx(self, rx: ScienceSerialRxPacket):
        # Ignore anythign with an error code
        if rx.error_code != 0:
            return
        # Iterate over observers
        for observer in self.observers:
            # If the rx packet has the same command word as this observer
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

class ActuatorQuality(RxObserver):
    def __init__(self, func_def, pub_tx, tx_packet_builder, update_method):
        super().__init__(func_def, pub_tx, None)
        self.tx_packet_builder = tx_packet_builder
        self.update_method = update_method

    def request(self, actuator_index):
        if self.observing:
            # print(f'Already observing on func_def {self.func_def["function_name"]}')
            return
        else:
            self.actuator_index = actuator_index
            self.pub_tx.publish( self.tx_packet_builder(self.actuator_index) )
            self.observing = True
            # print(f'Begin observing for index {actuator_index} on func_def {self.func_def["function_name"]}')

    def receive(self, rx):
        # Race Condition, dont flip these
        self.observing = False
        self.update_method(self.actuator_index, SMFL.get_return_data(self.func_def, rx))

class ActuatorPosition(ActuatorQuality):
    def __init__(self, pub_tx, actuator_state_manager):
        super().__init__(
            SMFL.get_function_by_function_name('get_actuator_position'),
            pub_tx,
            SMFL.get_tx_get_actuator_position,
            actuator_state_manager.update_position
        )

class ActuatorControl(ActuatorQuality):
    def __init__(self, pub_tx, actuator_state_manager):
        super().__init__(
            SMFL.get_function_by_function_name('get_actuator_control'),
            pub_tx,
            SMFL.get_tx_get_actuator_control,
            actuator_state_manager.update_control
        )

class ActuatorReserved(ActuatorQuality):
    def __init__(self, pub_tx, actuator_state_manager):
        super().__init__(
            SMFL.get_function_by_function_name('query_is_actuator_reserved'),
            pub_tx,
            SMFL.get_tx_query_is_actuator_reserved,
            actuator_state_manager.update_reserved
        )


class ActuatorStateManager():
    def __init__(self, pub_tx, pub_forward, observer_list):
        self.requests = []
        self.pending = 0
        self.pub_forward = pub_forward
        # Add the observers to the total observer list, give them a reference back here
        self.pos_obs = ActuatorPosition(pub_tx, self)
        observer_list.append(self.pos_obs)
        self.ctrl_obs = ActuatorControl(pub_tx, self)
        observer_list.append(self.ctrl_obs)
        self.res_obs = ActuatorReserved(pub_tx, self)
        observer_list.append(self.res_obs)

        self.index = None
        self.position = None
        self.control = None
        self.reserved = None

    def request(self, actuator_index):
        # Request an actuator update on index 
        if actuator_index in self.requests:
            # Ignore this request, already on stack
            return
        else:
            # print(f'Stacking request for act index {actuator_index}')
            self.requests.append(actuator_index)
            self.start_request_if_ready()

    def start_request_if_ready(self):
        if self.pending > 0 or len(self.requests) == 0:
            # Dont start a new request if one is already open or non available
            return
        else:
            # print(f'Submitting request for act index {self.requests[0]}')
            self.index = self.requests[0]
            self.pos_obs.request(self.index)
            self.ctrl_obs.request(self.index)
            self.res_obs.request(self.index)
            self.pending = 3
    
    def update_position(self, index, position):
        # Update the actuator state with the new position
        if index != self.index:
            raise ValueError(f"Returned actuator index does not match the current requested index, expected {self.index} got {index}")
        self.position = position
        # print(f'Received position value at index {index}')
        # print(f'Checking pos observer has resolved: {self.pos_obs.isObserving()}')
        self.reduce_pending()

    def update_control(self, index, control):
        # Update the actuator state with the new control
        if index != self.index:
            raise ValueError(f"Returned actuator index does not match the current requested index, expected {self.index} got {index}")
        self.control = control
        # print(f'Received control value at index {index}')
        # print(f'Checking ctrl observer has resolved: {self.ctrl_obs.isObserving()}')
        self.reduce_pending()

    def update_reserved(self, index, reserved):
        # Update the actuator state with the new reserved state
        if index != self.index:
            raise ValueError(f"Returned actuator index does not match the current requested index, expected {self.index} got {index}")
        self.reserved = reserved
        # print(f'Received reserved state at index {index}')
        # print(f'Checking res observer has resolved: {self.res_obs.isObserving()}')
        self.reduce_pending()

    def reduce_pending(self):
        self.pending -= 1
        if self.pending == 0:
            # print(f'Completed request at index {self.index}')
            # Remove the request from the stack once complete
            self.requests.pop(0)
            # Publish the actuator state update
            self.pub_forward.publish(
                ScienceActuatorState(
                    index = int(self.index),
                    position = int(self.position),
                    control = int(self.control),
                    reserved = bool(self.reserved)
                )
            )
            # Start the next request if there is one
            self.start_request_if_ready()


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
    def __init__(self, pub_tx, pub_response):
        super().__init__(SMFL.get_function_by_function_name('get_analog_sensor_raw'), pub_tx, pub_response, SMFL.get_tx_get_analog_sensor_raw)

class CalibratedAnalogSensorValues(AnalogSensorValues):
    def __init__(self, pub_tx, pub_response):
        super().__init__(SMFL.get_function_by_function_name('get_analog_sensor_calibrated'), pub_tx, pub_response, SMFL.get_tx_get_analog_sensor_calibrated)

class Spectrograph(RxObserver):
    def __init__(self, pub_tx, pub_forward):
        super().__init__(SMFL.get_function_by_function_name('return_spectrograph_data'), pub_tx, pub_forward)
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
            self.pub_tx.publish( SMFL.get_tx_sample_spectrograph(num_samples, sample_duration_ms, bulb_on) )
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
    def __init__(self, pub_tx, pub_forward):
        super().__init__(SMFL.get_function_by_function_name('return_ltr_data'), pub_tx, pub_forward)
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