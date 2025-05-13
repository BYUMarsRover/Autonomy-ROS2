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
from std_msgs.msg import Empty, String, Bool, UInt8
from science.function_mapping.function_map import ScienceModuleFunctionList as SMFL
from science.function_mapping.function_map import ScienceModuleFunctionListBuilder as SMFL_Builder
import struct
import time
from science.science_serial_interface import ScienceSerialInterface

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
        self.sub_get_actuator_state = self.create_subscription(UInt8, '/science_get_actuator_state', self.actuator_state_request, 10)
        self.sub_get_routine_status = self.create_subscription(Empty, '/science_get_routine_status', self.routine_status_request, 10)

        # Publisher to various response channels
        self.pub_analog_sensors = self.create_publisher(ScienceSensorValues, '/science_sensor_values', 10)
        self.pub_spectrograph = self.create_publisher(ScienceSpectroData, '/science_spectro_data', 10)
        self.pub_uv_sensor = self.create_publisher(ScienceUvData, '/science_uv_data', 10)
        self.pub_actuator_state = self.create_publisher(ScienceActuatorState, '/science_actuator_state_update', 10)
        self.pub_routine_status = self.create_publisher(String, '/science_routine_status_update', 10)

        # Publisher to the request TX Channel
        self.pub_tx = self.create_publisher(ScienceSerialTxPacket, '/science_serial_tx_request', 10)
        # Subscriber to the response RX Channel
        self.sub_rx = self.create_subscription(ScienceSerialRxPacket, '/science_serial_rx_packet', self.parse_rx, 10)

        # Complex Observer Types
        self.observers = []
        self.requests = []

    def sensor_request(self, msg: Bool):
        if msg.data == True:
            self.requests.append(self.CalibratedAnalogSensorValues(self, self.pub_analog_sensors))
        else:
            self.requests.append(self.RawAnalogSensorValues(self, self.pub_analog_sensors))

    def spectrograph_request(self, msg: Empty):
        self.requests.append(self.SpectrographRequest(self, self.pub_spectrograph))

    def uvsensor_request(self, msg: Empty):
        self.requests.append(self.UVSensorRequest(self, self.pub_uv_sensor))

    def routine_status_request(self, msg: Empty):
        self.get_logger().info("Routine Status Request")
        self.do_SMFL_async_call("query_routine_controller", [], lambda str: self.pub_routine_status.publish(String(data="Failed" if str is None else str )), 5)

    def actuator_state_request(self, msg: UInt8, pos=True, ctrl=True, res=True):
        self.requests.append(self.ActuatorStateRequest(self, self.pub_actuator_state, msg.data, pos, ctrl, res))

    def parse_rx(self, rx: ScienceSerialRxPacket):
        # Iterate over observers
        found = False
        # self.get_logger().error(f"Parsing RX packet with echo {rx.echo.tobytes()} and error code {rx.error_code} for {len(self.observers)} observers.")
        for observer in self.observers:
            if rx.error_code == 0 and observer.is_response(rx):
                try:
                    # self.get_logger().info(f"Observer {observer.func_name} matched!. Expected {observer.echo}, got {rx.echo.tobytes()}.")
                    observer.forward(rx)
                    self.observers.remove(observer)
                    found = True
                except Exception as e:
                    self.get_logger().error(f"Error in observer {observer.func_name}: {e}")
            elif observer.is_expired(rx.timestamp):
                observer.timeout()
                self.observers.remove(observer)
                self.get_logger().warn(f"Observer {observer.func_name} removed due to timeout.")
        # if found is False:
            # self.get_logger().warn(f"Received RX packet with no matching observer. Echo: {rx.echo.tobytes()}")
        
        # Perform a clean step
        self.clean_requests()

    def register_observer(self, func_name, echo, callback, timeout=5):
        # Register an observer to the list
        self.observers.append(self.ResponsePacketObserver(func_name, echo, callback, timeout))
        # self.get_logger().warn(f"Registered observer for {func_name} with echo {echo} and timeout {timeout} seconds.")
            
    class ResponsePacketObserver:
        def __init__(self, func_name, echo, callback, timeout=5):
            self.func_name = func_name
            self.echo = echo
            self.callback = callback
            self.expiration = time.time() + timeout

        def is_response(self, rx: ScienceSerialRxPacket):
            # Check if the response is valid
            return self.echo == rx.echo.tobytes()
        
        def is_expired(self, current_time):
            # Decrease the timeout value
            return current_time > self.expiration
    
        def forward(self, message):
            # Call the sucess callback
            print(f"Observer for {self.func_name} calling back...")
            self.callback(message)

        def timeout(self):
            # Call the timeout callback
            self.callback(None)

    def do_async_call(self, func_name, tx_packet, callback, timeout=5):
        # Call the SMFL function and get the return data asynchrously
        echo = ScienceSerialInterface.author_packet(tx_packet)
        echo = struct.pack('B' * len(echo), *echo)
        self.register_observer(func_name, echo, callback, timeout)
        self.pub_tx.publish(tx_packet)

    def do_SMFL_async_call(self, func_name, operand_list, callback, timeout=5):
        # Call the SMFL function and get the return data asynchrously
        func_def = SMFL.get_function_by_function_name(func_name)
        tx_packet = SMFL_Builder.build_tx_packet(func_def, operand_list)
        self.do_async_call(
            func_name,
            tx_packet,
            lambda rx: callback(None if rx is None else SMFL_Builder.get_return_data(func_def, rx)),
            timeout
        )

    def clean_requests(self):
        '''Cleans up any old unused requests that were mantained for callbacks'''
        for req in self.requests:
            if req.complete():
                req.clean_up()
                self.requests.remove(req)

    class Request:
        def __init__(self, request_manager):
            self.request_manager = request_manager

        def complete(self):
            # Check if the request is complete
            self.request_manager.get_logger().info(f"Request Complete route not implemented for {self.__class__.__name__}")
            return True
        
        def clean_up(self):
            # Ran when the request is begin deleted
            return
        
    class PolledRequest(Request):
        def __init__(self, request_manager, pub_forward, tx_start, func_name_return, poll_speed=1, attempts=10):
            super().__init__(request_manager)
            self.pub_forward = pub_forward
            self.func_name_return = func_name_return
            self.timer = None
            self.done = False
            self.attempts = attempts

            # Tell the spectrograph to start taking data
            self.request_manager.pub_tx.publish(tx_start)

            # Create a timer to poll the spectrograph data
            self.timer = request_manager.create_timer(poll_speed, 
                lambda: self.check_if_data_ready(poll_speed) # Check every second
            )

        def check_if_data_ready(self, poll_rate):
            # Check if the data is ready
            self.request_manager.do_SMFL_async_call(self.func_name_return, [], lambda data: self.receive(data), poll_rate)

        def receive(self, data):
            if data != None:
                self.publish(data)
                self.done = True
            self.attempts -= 1
            if self.attempts == 0:
                # Let it die
                self.done = True

        def complete(self):
            return self.done
        
        def clean_up(self):
            # Cleanup the timer
            if self.timer is not None:
                self.timer.destroy()
                self.timer = None

    class SpectrographRequest(PolledRequest):
        def __init__(self, request_manager, pub_forward, poll_speed=1, attempts=10, num_samples=1, sample_duration_ms=1000, bulb_on=True):
            super().__init__(request_manager, pub_forward, SMFL_Builder.get_tx_sample_spectrograph(num_samples, sample_duration_ms, bulb_on), "return_spectrograph_data", poll_speed, attempts)

        def publish(self, data):
            self.pub_forward.publish(
                ScienceSpectroData(
                    values = data
                )
            )

    class UVSensorRequest(PolledRequest):
        def __init__(self, request_manager, pub_forward, poll_speed=1, attempts=10):
            super().__init__(request_manager, pub_forward, SMFL_Builder.get_tx_sample_ltr(), "return_ltr_data", poll_speed, attempts)

        def publish(self, data):
            self.pub_forward.publish(
                ScienceUvData(
                    als=data[1],  # Second Float
                    uv=data[0]    # First Float
                )
            )

    class ActuatorStateRequest(Request):
        def __init__(self, request_manager, pub_forward, index, pos=True, ctrl=True, res=True):
            super().__init__(request_manager)
            self.index = index
            self.pub_forward = pub_forward
            self.position = None
            self.control = None
            self.reserved = None

            self.request_manager.get_logger().info(f"Actuator State Request Created for {index} with pos={pos}, ctrl={ctrl}, res={res}")

            self.pending = 0
            if pos:
                self.pending += 1
                self.request_manager.do_SMFL_async_call("get_actuator_position", [index], lambda pos: self.update_position(index, pos))
            if ctrl:
                self.pending += 1
                self.request_manager.do_SMFL_async_call("get_actuator_control", [index], lambda ctrl: self.update_control(index, ctrl))
            if res:
                self.pending += 1
                self.request_manager.do_SMFL_async_call("query_is_actuator_reserved", [index], lambda res: self.update_reserved(index, res))
        
        def update_position(self, index, position):
            # Update the actuator state with the new position
            if index != self.index:
                raise ValueError(f"Returned actuator index does not match the current requested index, expected {self.index} got {index}")
            self.position = position
            self.reduce_pending()

        def update_control(self, index, control):
            # Update the actuator state with the new control
            if index != self.index:
                raise ValueError(f"Returned actuator index does not match the current requested index, expected {self.index} got {index}")
            self.control = control
            self.reduce_pending()

        def update_reserved(self, index, reserved):
            # Update the actuator state with the new reserved state
            if index != self.index:
                raise ValueError(f"Returned actuator index does not match the current requested index, expected {self.index} got {index}")
            self.reserved = reserved
            self.reduce_pending()

        def reduce_pending(self):
            self.pending -= 1
            if self.pending == 0:
                if self.position is None or self.control is None or self.reserved is None:
                    return
                else:
                    self.request_manager.get_logger().info(f"Actuator State Request for {self.index} completed!")
                    self.pub_forward.publish(
                        ScienceActuatorState(
                            index = int(self.index),
                            position = int(self.position),
                            control = int(self.control),
                            reserved = bool(self.reserved)
                        )
                    )

        def complete(self):
            return self.pending == 0

    class AnalogSensorValues(Request):
        def __init__(self, request_manager, pub_forward, func_name):
            super().__init__(request_manager)
            self.pub_forward = pub_forward
            self.func_name = func_name
            self.temperature = None
            self.humidity = None
            self.send()
        
        def send(self):
            self.request_manager.do_SMFL_async_call(self.func_name, [TEMP_SENSOR_INDEX], lambda data: self.receiveTemp(data))
            self.request_manager.do_SMFL_async_call(self.func_name, [HUM_SENSOR_INDEX], lambda data: self.receiveHumidity(data))

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

        def complete(self):
            return self.temperature is not None and self.humidity is not None

    class RawAnalogSensorValues(AnalogSensorValues):
        def __init__(self, request_manager, pub_forward):
            super().__init__(request_manager, pub_forward, "get_analog_sensor_raw")

    class CalibratedAnalogSensorValues(AnalogSensorValues):
        def __init__(self, request_manager, pub_forward):
            super().__init__(request_manager, pub_forward, "get_analog_sensor_calibrated")



def main(args=None):
    rclpy.init(args=args)
    science_request_manager = ScienceRequestManager()

    science_request_manager.get_logger().info('Science Request Manager Online')
    rclpy.spin(science_request_manager)
    science_request_manager.destroy_node()
    rclpy.shutdown

if __name__ == '__main__':
    main()