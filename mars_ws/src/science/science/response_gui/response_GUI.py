#!/usr/bin/python3

from PyQt5 import QtWidgets, uic, QtGui
from PyQt5.QtCore import Qt
import science.response_gui.resources
from python_qt_binding.QtCore import QObject, Signal
import rclpy
from rover_msgs.msg import ScienceSerialRxPacket
from std_msgs.msg import UInt8MultiArray
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import matplotlib.pyplot as plt
import numpy as np
import datetime

import os
import sys

class Signals(QObject):
    science_rx_signal = Signal()
    science_tx_signal = Signal()
    # sensor_save_signal = Signal(ScienceSaveSensor)
    # FAD_save_signal = Signal(ScienceSaveFAD)
    # notes_save_signal = Signal(ScienceSaveNotes)
    # fad_intensity_signal = Signal(ScienceFADIntensity)

class science_response_GUI(Node):
    def __init__(self):
        
        super().__init__('science_response_GUI')
        self.qt = QtWidgets.QWidget()

        ui_file_path = os.path.join(
            get_package_share_directory('science'),
            'response_gui',
            'response_packet.ui'
            )
        
        self.icon_check = QtGui.QIcon(":/images/accept.png")
        self.icon_message = QtGui.QIcon(":/images/bell.png")
        self.icon_warning = QtGui.QIcon(":/images/warning.png")
        self.icon_error = QtGui.QIcon(":/images/close.png")
        self.icons = [self.icon_check, self.icon_message, self.icon_warning, self.icon_error]

        uic.loadUi(ui_file_path, self.qt)
        self.task_launcher_init()

        self.response_packet_list = []

        self.receive_response_packet(
            ScienceSerialRxPacket(
                timestamp = 8653135.12345,
                echo = [0x53, 0x00, 0x00, 0x4D],
                error_code = 0,
                message = [0xb7, 0xb4, 0xd5, 0x56, 0x5b, 0x5b, 0x2a, 0x56, 0xad, 0x54, 0xad, 0x15, 0xb5\
, 0x48, 0x50, 0x91, 0xc0, 0x4c, 0x26, 0x24]
            )
        )

        # Create Publisher
        # self.science_serial_tx_request_pub = self.create_publisher(ScienceSerialTxPacket, '/science_serial_tx_request', 10)

        # # Create Subscriptions
        self.science_serial_rx_sub = self.create_subscription(ScienceSerialRxPacket, '/science_serial_rx_packet', self.receive_response_packet, 10)
        # self.science_serial_rx_sub = self.create_subscription(UInt8MultiArray, '/science_serial_rx_notification', self.receive_rx_notification, 10)

        # # Ensure empty
        # self.clear_rxtx_windows()

        self.qt.show() # Show the GUI

    def extract_timestamp(self, msg: ScienceSerialRxPacket):
        return datetime.datetime.fromtimestamp(msg.timestamp).__str__()

    def receive_response_packet(self, msg: ScienceSerialRxPacket):

        # Save the message for future reference
        index = len(self.response_packet_list)
        self.response_packet_list.append(msg)

        # choose the icon
        if msg.error_code == 0:
            icon = self.icon_message
        elif msg.error_code == 1:
            icon = self.icon_warning
        else:
            icon = self.icon_error

        # Get name from ascii contents of message
        preview_length = 10
        ascii_text = ''.join(chr(byte) if 32 <= byte <= 126 else '#' for byte in msg.message)

        # Add to the list widget
        self.add_response_message_widget(
            icon,
            f"{ascii_text[:preview_length]} - {self.extract_timestamp(msg)}",
            index
        )


    def add_response_message_widget(self, icon, name, index):
        item = QtWidgets.QListWidgetItem(name)
        item.setIcon(icon)  # Set the icon for the item
        item.setData(Qt.UserRole, {"index": index})
        self.qt.packet_browser.addItem(item)  # Add the item to the QListWidget

    def select_response_packet(self):
        item = self.qt.packet_browser.currentItem()
        if not item is None:
            print(item.text())

    def task_launcher_init(self):
        self.signals = Signals()

        self.qt.packet_browser.itemSelectionChanged.connect(self.select_response_packet)

        # # Clear Button
        # self.qt.pushButton_clear.clicked.connect(self.clear_rxtx_windows)

        # # Set up form validation
        # self.qt.lineEdit_function_addr.setValidator(QtGui.QIntValidator(0, 31)) # 5 bits

        # # Submit Form Button
        # self.qt.pushButton_submit_form.clicked.connect(self.submit_packet_form)
        # self.qt.pushButton_hex_manual.clicked.connect(self.submit_manual_hexdecimal_form)
        # self.qt.pushButton_ascii_manual.clicked.connect(self.submit_manual_ascii_form)

        # self.qt.pushButton_fad.clicked.connect(self.fad_detector_get_point)
        # self.qt.pushButton_fad_calibration.clicked.connect(self.save_fad)

        # self.qt.pushButton_moisture.clicked.connect(lambda: self.graph_sensor_values(0))
        # self.qt.pushButton_temperature.clicked.connect(lambda: self.graph_sensor_values(1))
        # self.qt.pushButton_fad_graph.clicked.connect(lambda: self.graph_sensor_values(2))

        # self.qt.pushButton_moisture_2.clicked.connect(lambda: self.estimate_reading(0))
        # self.qt.pushButton_temperature_2.clicked.connect(lambda: self.estimate_reading(1))
        # self.qt.pushButton_fad_estimate.clicked.connect(lambda: self.estimate_reading(2))

        # self.qt.moist_radio.toggled.connect(lambda: self.toggle_sensor_save(0))  # moist
        # self.qt.temp_radio.toggled.connect(lambda: self.toggle_sensor_save(1))  # temp
        # self.qt.fad_radio.toggled.connect(lambda: self.toggle_sensor_save(2))  # fad

        # self.qt.lcd_site_num.display(self.site_number)
        # self.qt.pushButton_change_site.clicked.connect(self.increment_site_number)

        # self.pub_save_sensor = self.create_publisher(ScienceSaveSensor, '/science_save_sensor', 1) #figure this out
        # self.pub_save_notes = self.create_publisher(ScienceSaveNotes, '/science_save_notes', 1)
        # self.pub_save_fad = self.create_publisher(ScienceSaveFAD, '/science_save_fad', 1)

        # self.signals.sensor_signal.connect(self.update_sensor_values)
        # # self.signals.auger_position.connect(self.update_auger_position)
        # self.signals.sensor_save_signal.connect(self.pub_save_sensor.publish)
        # self.signals.FAD_save_signal.connect(self.pub_save_fad.publish)
        # self.signals.notes_save_signal.connect(self.pub_save_notes.publish)
        # self.signals.fad_intensity_signal.connect(self.update_fad_intensity_value)

        # self.science_sensor_values = self.create_subscription(ScienceSensorValues, '/science_sensor_values', self.signals.sensor_signal.emit, 10)
        # # self.science_auger_position = self.create_subscription(ScienceToolPosition, '/science_auger_position', self.signals.auger_position.emit, 10)
        # self.science_fad_calibration = self.create_subscription(ScienceFADIntensity, '/science_fad_calibration', self.signals.fad_intensity_signal.emit, 10)
        # self.rover_state_singleton = self.create_subscription(RoverStateSingleton, '/odometry/rover_state_singleton', self.update_pos_vel_time, 10)

    def append_text_to_label(self, label, text):
        label.setText(label.text() + text)

    def append_serial_traffic(self, data, hex_label, ascii_label):
        hex_text = ' '.join(f'0x{byte:02x}' for byte in data) + ' '
        ascii_text = ' '.join(chr(byte) if 32 <= byte <= 126 else '.' for byte in data) + ' '
        self.append_text_to_label(hex_label, hex_text)
        self.append_text_to_label(ascii_label, ascii_text)

    def update_rx(self, data):
        self.append_serial_traffic(data, self.qt.label_rx_hex, self.qt.label_rx_ascii)

    def update_tx(self, data):
        self.append_serial_traffic(data, self.qt.label_tx_hex, self.qt.label_tx_ascii)

    def clear_rxtx_windows(self):
        self.qt.label_rx_ascii.clear()
        self.qt.label_rx_hex.clear()
        self.qt.label_tx_ascii.clear()
        self.qt.label_tx_hex.clear()

    def form_command_word(self):
        # Get the function adress from the form
        text = self.qt.lineEdit_function_addr.text()
        if text == "":
            self.get_logger().error("Function address is required.")
            return None
        command_word = int(self.qt.lineEdit_function_addr.text())

        # Set the three most significant bits based on the checkboxes
        command_word |= (self.qt.checkBox_com.isChecked() << 7)
        command_word |= (self.qt.checkBox_ovr.isChecked() << 6)
        command_word |= (self.qt.checkBox_ack.isChecked() << 5)

        return command_word
    
    def read_hex_values(self, lineEdit):
        """
        Reads a series of hex values from a QLineEdit and returns them as an array of uint8_t integers.
        """
        hex_string = lineEdit.text()
        try:
            # Split the input string by spaces and convert each hex value to an integer
            hex_values = [int(value, 16) for value in hex_string.split()]
            # Ensure all values fit within uint8_t range (0-255)
            if all(0 <= value <= 255 for value in hex_values):
                return hex_values
            else:
                raise ValueError("One or more values are out of range for uint8_t.")
        except ValueError as e:
            self.get_logger().error(f"Invalid hex input: {e}")
            return []

    def submit_packet_form(self):

        print("form")

        # Get command word
        command_word = self.form_command_word()
        if command_word is None:
            return
        
        operands = self.read_hex_values(self.qt.lineEdit_operands)

        # Publish to be handled by the science_serial node
        self.science_serial_tx_request_pub.publish(
            ScienceSerialTxPacket(
                command_word=command_word,
                operands=operands
            )
        )

    def submit_manual_hexdecimal_form(self):
        # Publish to be handled by the science_serial node
        print("maunal hex")
        self.science_serial_tx_request_pub.publish(
            ScienceSerialTxPacket(packet=self.read_hex_values(self.qt.lineEdit_hex_manual))
        )

    def submit_manual_ascii_form(self):
        print("maunal ascii")
        # Convert ASCII string input to its hexadecimal representation
        ascii_string = self.qt.lineEdit_ascii_manual.text()
        hex_values = [ord(char) for char in ascii_string]  # Convert each character to its ASCII value
        
        # Publish to be handled by the science_serial node
        self.science_serial_tx_request_pub.publish(
            ScienceSerialTxPacket(packet=hex_values)
        )

    def receive_tx_notification(self, msg: UInt8MultiArray):
        self.update_tx(msg.data)

    def receive_rx_notification(self, msg: UInt8MultiArray):
        self.update_rx(msg.data)
    
    # def toggle_sensor_save(self, p):
    #     """
    #     Called when any sensor radio button is called (moist, temp, fad)
    #     Tells science data saver to start saving values or to finish saving values.
    #     p: sensor number, [0: moist, 1: temp, 2: fad]
    #     """

    #     print('Toggling Saving Sensor', p)
    #     self.sensor_saving[p] = not self.sensor_saving[p]
    #     if (p == 0): # Moisture radio button
    #         sensor_message = ScienceSaveSensor(site=self.site_number, position=p, observed_value=float(self.qt.lineEdit_moisture.text()), save=self.sensor_saving[p])
    #     elif (p == 1): # Temp radio button
    #         sensor_message = ScienceSaveSensor(site=self.site_number, position=p, observed_value=float(self.qt.lineEdit_temperature.text()), save=self.sensor_saving[p])
    #     else:  # fad radio button, probably going to end up being useless.
    #         if self.qt.fad_radio.isChecked():
    #             self.fad_timer = self.create_timer(self.save_interval, self.stop_fad_saver)
    #             print("made timer for FAD")
    #         if (self.qt.lineEdit_fad.text() != ''):
    #             sensor_message = ScienceSaveSensor(site=self.site_number, position=p, observed_value=float(self.qt.lineEdit_fad.text()), save=self.sensor_saving[p])
    #         else:
    #             #Find how do get logger
    #             sensor_message = ScienceSaveSensor(site=self.site_number, position=p, observed_value=3.141592, save=self.sensor_saving[p])

    #     self.signals.sensor_save_signal.emit(sensor_message)

    # def save_fad(self):
    #     self.sensor_saving[2] = True #2 is the FAD value, consider changing this to a const
    #     sensor_message = ScienceSaveFAD(site=self.site_number, intensity=int(self.qt.le_fad.text()), observed=float(self.qt.lineEdit_fad.text()))
    #     self.signals.FAD_save_signal.emit(sensor_message)

    # def stop_temp_saver(self):
    #     self.temp_timer.cancel()
    #     self.qt.temp_radio.setChecked(False)

    # def stop_moist_saver(self):
    #     self.moisture_timer.cancel()
    #     self.qt.moist_radio.setChecked(False)

    # def stop_fad_saver(self):
    #     # self.fad_timer.cancel()
    #     print("Attempting to uncheck the FAD")
    #     self.fad_timer = None
    #     self.qt.fad_radio.setChecked(False)

    # def increment_site_number(self):
    #     """
    #     Increments the site number
    #     """
    #     self.site_number += 1
    #     self.qt.lcd_site_num.display(self.site_number)

    # def save_notes(self):
    #     """
    #     Saves the current notes under the given site.
    #     """
    #     print('Saving notes.')
    #     self.save_notes_msg = ScienceSaveNotes()
    #     self.signals.notes_save_signal.emit(self.save_notes_msg)
    #     print('Notes sent.')

    # def update_sensor_values(self, msg):
    #     # print("updating LCD displays")
    #     temperature = msg.temperature
    #     moisture = msg.moisture

    #     self.qt.lcd_moist.display(moisture)
    #     self.qt.lcd_temp.display(temperature)

    # def graph_sensor_values(self, position):
    #     manual_points = []
    #     analog_vals = []
    #     coefficients_path = ""
    #     coefficients_file = ""

    #     match(position):
    #         case 0:
    #             file_name = "moisture-plot-1.txt"
    #             coefficients_file = "moisture_coefficients.txt"
    #         case 1:
    #             file_name = "temperature-plot-1.txt"
    #             coefficients_file = "temperature_coefficients.txt"
    #         case 2:
    #             file_name = "fad-plot-1.txt"
    #             coefficients_file = "fad_coefficients.txt"
    #         case _: #Wildcard, acts like else
    #             print("Err: this sensor does not have data to graph")
    #             return
    #     file_path = os.path.join(self.science_data_path, file_name)
    #     coefficients_path = os.path.join(self.science_data_path, coefficients_file)

    #     #Check file existence
    #     if not os.path.exists(file_path):
    #         print("Err: file does not exist")
    #         return

    #     with open(file_path, 'r') as f:
    #         for line in f:
    #             split = line.split()
    #             manual_points.append(float(split[0]))
    #             reading_series =[]
    #             for i in split[1:]:
    #                 reading_series.append(float(i))
    #                 #Normalize the values form zero to 1.
    #                 reading_series[-1] = reading_series[-1]/1023
    #             analog_vals.append(reading_series)
            
    #         #Show an updated graph with the new point
    #         dummy_manuals = []
    #         for i in range(len(analog_vals)):
    #             for j in analog_vals[i]:
    #                 dummy_manuals.append(manual_points[i])
    #         dummy_analog = []
    #         for i in analog_vals:
    #             for j in i:
    #                 dummy_analog.append(j)

    #         plt.scatter(dummy_analog,dummy_manuals)
    #         plt.pause(0.5)
    #         # Have it ask you to save the point or delete after showing you an updated graph.
    #         # Need to decide if this is useful or not
    #         # keep = "y"
    #         # if not (keep == "y" or keep =="Y" or keep == "[Y]" or keep == "[y]" or keep == ""):
    #         #     manual_points.pop()
    #         #     analog_vals.pop()
    #         #     plt.cla()
    #         #     plt.scatter(dummy_analog,dummy_manuals)
    #         #     plt.xlabel("Arduino Digital Readout")
    #         #     plt.ylabel("Reference Temperature (deg C)")
    #         #     plt.pause(0.5)
    #     #Add something so you can decide what order polynomial you want.
    #     order = int(input("What order polynomial do you want to fit? [0 - 6]\n"))
    #     P0 = np.zeros((1,6-order))

    #     #Plot the points alongside the polyfit.
    #     analog_vals = np.array(dummy_analog)
    #     manual_points = np.array(dummy_manuals)
    #     P1 = np.polyfit(analog_vals, manual_points, order)
    #     P = np.concatenate((P0,P1),axis=None)
    #     x = np.linspace(0,1,500)
    #     poly_y = P[0]*x**6+P[1]*x**5+P[2]*x**4+P[3]*x**3+P[4]*x**2+P[5]*x + P[6]
    #     plt.figure()
    #     plt.scatter(analog_vals, manual_points, label="input data")
    #     plt.xlabel("Arduino Digital Readout")
    #     plt.ylabel("Reference Temperature (deg C)")
    #     plt.plot(x, poly_y, label="polynomial fit")
    #     plt.legend()
    #     plt.show()

    #     with open(coefficients_path, 'w') as f:
    #         line = ''
    #         for i in P:
    #             line += str(i)
    #             line += " "
    #         f.write(line)

    # def estimate_reading(self, position):
    #     match(position):
    #         case 0:
    #             coefficients_file = "moisture_coefficients.txt"
    #             try:
    #                 val = self.qt.lineEdit_moisture_2.text()
    #                 x = float(val)/1023
    #             except (ValueError):
    #                 self.get_logger().error(f"{val} is not a valid number")
    #                 return
    #         case 1:
    #             coefficients_file = "temperature_coefficients.txt"
    #             try:
    #                 val = self.qt.lineEdit_moisture_2.text()
    #                 x = float(val)/1023
    #             except (ValueError):
    #                 self.get_logger().error(f"{val} is not a valid number")
    #                 return
    #         case 2:
    #             coefficients_file = "fad_coefficients.txt"
    #             try:
    #                 val = self.qt.lineEdit_moisture_2.text()
    #                 x = float(val)/1023
    #             except (ValueError):
    #                 self.get_logger().error(f"{val} is not a valid number")
    #                 return
    #         case _: #Wildcard, acts like else. This should never happen.
    #             print("Err: this sensor does not have data to graph")
    #             return
    #     coefficients_path = os.path.join(self.science_data_path, coefficients_file)
        
    #     with open(coefficients_path, 'r') as f:
    #         coefs = f.read().split()
    #         result = 0
    #         for i in range(len(coefs)):
    #             result += (float(coefs[i]) * (float(x)**i))
    #     sensor = "None"
    #     if position == 0: sensor = "Moisture"
    #     elif position == 1: sensor = "Temperature"
    #     elif position == 2: sensor = "FAD"
    #     else: sensor = "Unknown sensor"
    #     self.qt.textBrowser.setPlainText(sensor + f" reading is {result}")
        
    # def update_fad_intensity_value(self, msg):
    #     #Insert code to send reading to save?
    #     # print('Displaying intensity!', msg)
    #     self.qt.le_fad.setText(str(msg.intensity_avg))

    # def update_pos_vel_time(self, msg):
    #     altitude = f'{msg.gps.altitude} ft'
    #     heading = f'{msg.map_yaw}'
    #     coordinates = f'{msg.gps.latitude}, {msg.gps.longitude}'

    #     print(f'(lat, long, altitude, heading): ({coordinates}, {altitude}, {heading})')

    #     self.qt.lbl_altitude.setText(altitude)
    #     self.qt.lbl_heading.setText(heading)
    #     self.qt.lbl_coordinates.setText(coordinates)

    # # def update_auger_position(self, msg):
    # #     """
    # #     Updates the augur display.

    # #     This is like this because the photoresistors are backwards on the board.
    # #     """
    # #     if msg.position == 0:
    # #         self.qt.lcd_auger.display(2)
    # #     elif msg.position == 1:
    # #         self.qt.lcd_auger.display(1)
    # #     else:
    # #         self.qt.lcd_auger.display(-1)

    # def fad_detector_get_point(self, event=None): #Written by chat
    #     print('Calibrate FAD')
    #     if not self.cli.service_is_ready():
    #         self.get_logger().error("Camera control service is not available. Try again in a bit")
    #         return

    #     self.req = CameraControl.Request()
    #     self.req.camera.client_address = "{}@{}".format(os.getlogin(), self.base_ip)
    #     self.req.camera.camera_name = 'fadCam'
    #     self.req.site_name = 'fad_calibration'
    #     self.req.calibrate = True

    #     # Call the service asynchronously
    #     self.future = self.cli.call_async(self.req)
    #     self.future.add_done_callback(self.handle_fad_response)

    # def handle_fad_response(self, future): #written by chat
    #     try:
    #         response = future.result()
    #         self.get_logger().info(f"FAD Intensity: {response.intensity}")
    #         self.update_fad_intensity_value(response)
    #     except Exception as e:
    #         self.get_logger().error(f"Service call failed: {str(e)}")


    #     # print('Calibrate FADD')
    #     # site_name = 'fad_calibration'
    #     # camera = Camera()
    #     # camera.client_address = "{}@{}".format(os.getlogin(), self.base_ip)
    #     # camera.camera_name = 'fadCam'
    #     # response = self.camera_control(camera=camera, site_name=site_name, calibrate=True)

    #     # self.update_fad_intensity_value(response.intensity)

    # def get_base_ip(self):
    #     ip = os.getenv("BASE_ADDRESS")
    #     if ip is None:
    #         ip = "192.168.1.65"
    #     return ip
    
def main(args=None):
    rclpy.init(args=args)
    
    # Create the Qt application
    app = QtWidgets.QApplication(sys.argv)
    gui = science_response_GUI()  # Initialize your GUI

    # Start the ROS 2 spin loop in a separate thread or via a timer
    def spin_ros():
        rclpy.spin_once(gui, timeout_sec=0.1)  # This will process ROS callbacks

    # Set up a timer to periodically call spin_ros inside the Qt event loop
    timer = gui.qt.startTimer(100)  # Spin ROS every 100ms
    
    # Override the timer event to call spin_ros
    def timer_event(event):
        spin_ros()  # Call spin_ros to process any incoming ROS messages

    gui.qt.timerEvent = timer_event  # Assign the timer event handler

    # Start the Qt event loop
    app.exec_()

    rclpy.shutdown()


if __name__ == "__main__":
    main()