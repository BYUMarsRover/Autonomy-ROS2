#!/usr/bin/python3

from PyQt5 import QtWidgets, uic
from PyQt5.QtCore import QObject, pyqtSignal, QTimer, Qt, QAbstractTableModel
from PyQt5.QtWidgets import QApplication, QTableView
from python_qt_binding.QtCore import QObject, Signal
import rclpy
from std_msgs.msg import Empty, Bool, Float32
from rover_msgs.srv import CameraControl
from rover_msgs.msg import ScienceSensorValues, ScienceSaveSensor, ScienceSaveNotes, ScienceSaveFAD, ScienceFADIntensity, Camera, RoverStateSingleton, ScienceSpectroData, ScienceUvData, ScienceSaveSpectro
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import matplotlib.pyplot as plt
import numpy as np
import subprocess
import time

import os
import sys

ANALOG_SENSOR_QUERY_RATE_MS = 1000
LTR_390_QUERY_RATE_MS = 1000

class Signals(QObject):
    sensor_signal = Signal(ScienceSensorValues)
    auger_position = Signal(Bool)
    sensor_save_signal = Signal(ScienceSaveSensor)
    FAD_save_signal = Signal(ScienceSaveFAD)
    notes_save_signal = Signal(ScienceSaveNotes)
    fad_intensity_signal = Signal(ScienceFADIntensity)

class WavelengthTableModel(QAbstractTableModel):
    def __init__(self, wavelengths, values):
        super().__init__()
        self.wavelengths = wavelengths
        self.values = values

    def rowCount(self, parent=None):
        return len(self.wavelengths)

    def columnCount(self, parent=None):
        return 2

    def data(self, index, role):
        if role == Qt.DisplayRole:
            if index.column() == 0:
                return f"{self.wavelengths[index.row()]} nm"
            elif index.column() == 1:
                return f"{self.values[index.row()]:.4f}"
        return None

    def headerData(self, section, orientation, role):
        if role == Qt.DisplayRole:
            if orientation == Qt.Horizontal:
                return ["Wavelength (nm)", "Value"][section]
            elif orientation == Qt.Vertical:
                return str(section + 1)
        return None
    
    def updateData(self, new_values):
        if len(new_values) != len(self.wavelengths):
            print("Error: Data length mismatch")
            return
        self.values = new_values
        self.layoutChanged.emit() 

class science_GUI(Node):
    def __init__(self):
        
        super().__init__('science_GUI')
        self.qt = QtWidgets.QMainWindow()

        ui_file_path = os.path.join(
            get_package_share_directory('science'),
            'gui',
            'science_GUI.ui'
            )

        uic.loadUi(ui_file_path, self.qt)
        self.qt.show() # Show the GUI

        self.base_ip = self.get_base_ip()
        self.cli = self.create_client(CameraControl, 'camera_control')
        
        # Set up the spectrometer data table
        wavelengths = [410, 435, 460, 485, 510, 535, 560, 585, 610, 645, 680, 705, 730, 760, 810, 860, 900, 940]
        if len(wavelengths) != 18:
            raise ValueError("Wavelengths list must contain 18 elements.")
        values = [0 for i in range(18)]

        # Set Model to QTableView
        self.spectro_table = WavelengthTableModel(wavelengths, values)
        self.qt.tableView.setModel(self.spectro_table)

        # Notes Data
        self.site_notes = {}

        # Custom data to handle sensor requests
        self.measurement_temperature = TemperatureMS(self, period_ms=ANALOG_SENSOR_QUERY_RATE_MS, enabled=False, saving=False)
        self.measurement_moisture = MoistureMS(self, period_ms=ANALOG_SENSOR_QUERY_RATE_MS, enabled=False, saving=False)
        self.measurement_uv = UVIndexMS(self, period_ms=LTR_390_QUERY_RATE_MS, enabled=False, saving=False)
        self.measurement_ambient = AmbientLightIndexMS(self, period_ms=LTR_390_QUERY_RATE_MS, enabled=False, saving=False)

        self.site_number = 1
        self.init_publishers()
        self.create_signals()
        self.init_subscriptions()
        self.connect_ui_elements()
        self.setup_menu_bar()
        self.temperature_coefficients = [[],[],[],[],[],[]]
        self.moisture_coefficients = [[],[],[],[],[],[]]

        self.science_data_path = os.path.expanduser("~/science_data/site-1")

        # Read in coefficients. 
        try:
            moisture_path = os.path.join(self.science_data_path, "moisture_coefficients.txt")
            temp_path = os.path.join(self.science_data_path, "temp_coefficients.txt")
            if os.path.exists(moisture_path):
                with open(moisture_path, 'r') as f:
                    moisture_values = f.readlines()
                    print(moisture_values)
                    moisture_values = moisture_values[0].split()
                    for i in range(len(moisture_values)):
                        self.moisture_coefficients[i] = moisture_values[i]
            else:
                self.moisture_coefficients = []
                print("Moisture coefficients file does not exist. Please use the show graph button to store coefficients.")
            if os.path.exists(temp_path):
                with open(temp_path, 'r') as f:
                    temp_values = f[1].split()
                    for i in range(len(temp_values)):
                        self.temperature_coefficients[i] = temp_values[i]
            else:
                self.temperature_coefficients = []
                print("Temperature coefficients file does not exist. Please use the show graph button to store coefficients.")
        except Exception as e:
            self.get_logger().error(f"Error reading coefficients: {str(e)}")
            
    

    def setup_menu_bar(self):
        # Launch the GUI when the menu item is clicked
        self.qt.actionLaunch_Function_Library.triggered.connect(lambda: self.launch_gui("science_debug"))
        self.qt.actionLaunch_RX_TX_Monitor.triggered.connect(lambda: self.launch_gui("science_rxtx"))
        self.qt.actionLaunch_Response_Parse.triggered.connect(lambda: self.launch_gui("science_response"))
        self.qt.actionLaunch_Actuator_Command.triggered.connect(lambda: self.launch_gui("science_routine"))
        
    def launch_gui(self, executable):
        """
        Launch the response_gui node as a subprocess when the menu item is clicked.
        """
        try:
            # Launch the response_gui node using ros2 run
            subprocess.Popen(["ros2", "run", "science", executable])
            self.get_logger().info(f"Launched gui node {executable}.")
        except Exception as e:
            self.get_logger().error(f"Failed to launch gui: {str(e)}")

    def should_use_module_eeprom_calibration(self):
        return self.qt.actionRead_Calibrated_from_Module.isChecked()
    
    def init_publishers(self):
        self.pub_save_sensor = self.create_publisher(ScienceSaveSensor, '/science/save_sensor', 1) #figure this out
        self.pub_save_notes = self.create_publisher(ScienceSaveNotes, '/science/save_notes', 1)
        self.pub_save_fad = self.create_publisher(ScienceSaveFAD, '/science/save_fad', 1)
        self.pub_save_spectro = self.create_publisher(ScienceSaveSpectro, '/science/save_spectro', 1) #figure this out
        
        self.pub_get_spectro = self.create_publisher(Empty, '/science/spectro_request', 1)
        self.pub_get_uv = self.create_publisher(Empty, '/science/uv_request', 1)
        self.pub_get_analog_sensors = self.create_publisher(Bool, '/science/sensor_request', 1)

        self.pub_get_auger_position = self.create_publisher(Empty, '/science/auger_position', 1)

        # Kill Switch
        self.pub_kill_switch = self.create_publisher(Empty, "/science/emergency_stop", 10)

        # Calibrate UV
        self.pub_calibrate_uv = self.create_publisher(Float32, "/science/calibrate_uv", 10)

    def init_subscriptions(self):
        self.science_sensor_values = self.create_subscription(ScienceSensorValues, '/science/sensor_values', self.update_analog_sensor_values, 10)
        self.science_auger_position = self.create_subscription(Bool, '/science/using_probe', self.signals.auger_position.emit, 10)
        self.science_fad_calibration = self.create_subscription(ScienceFADIntensity, '/science/fad_calibration', self.signals.fad_intensity_signal.emit, 10)
        self.rover_state_singleton = self.create_subscription(RoverStateSingleton, '/odometry/rover_state_singleton', self.update_pos_vel_time, 10)
        self.sub_spectro = self.create_subscription(ScienceSpectroData, '/science/spectro_data', self.update_spectro_data, 10)
        self.sub_uv = self.create_subscription(ScienceUvData, '/science/uv_data', self.update_ltr_sensor_values, 10)

    def create_signals(self):
        self.signals = Signals()

        self.signals.auger_position.connect(self.update_current_tool)
        self.signals.sensor_save_signal.connect(self.pub_save_sensor.publish)
        self.signals.FAD_save_signal.connect(self.pub_save_fad.publish)
        self.signals.notes_save_signal.connect(self.pub_save_notes.publish)
        self.signals.fad_intensity_signal.connect(self.update_fad_intensity_value)

    def connect_ui_elements(self):

        self.qt.pushButton_save_notes.clicked.connect(self.save_notes)
        self.qt.pushButton_fad.clicked.connect(self.fad_detector_get_point)
        # self.qt.pushButton_fad_calibration.clicked.connect(self.save_fad)

        self.qt.pushButton_moisture.clicked.connect(lambda: self.graph_sensor_values(0))
        self.qt.pushButton_temperature.clicked.connect(lambda: self.graph_sensor_values(1))
        self.qt.pushButton_fad_graph.clicked.connect(lambda: self.graph_sensor_values(2))

        self.qt.pushButton_moisture_2.clicked.connect(lambda: self.estimate_reading(0))
        self.qt.pushButton_temperature_2.clicked.connect(lambda: self.estimate_reading(1))
        self.qt.pushButton_fad_estimate.clicked.connect(lambda: self.estimate_reading(2))

        self.qt.pushButton_spectro.clicked.connect(lambda: self.fetch_spectro_data())
        self.qt.pushButton_spectro_save.clicked.connect(lambda: self.save_spectro_values())
        self.qt.pushButton_uv.clicked.connect(lambda: self.fetch_uv_data())
        self.qt.pushButton_analog.clicked.connect(lambda: self.fetch_analog_sensors_data())

        # Saving toggles
        self.qt.saving_temp_checkBox.stateChanged.connect(lambda state: self.measurement_temperature.set_saving(state == Qt.Checked))
        self.qt.saving_moist_checkBox.stateChanged.connect(lambda state: self.measurement_moisture.set_saving(state == Qt.Checked))
        self.qt.saving_uv_checkBox.stateChanged.connect(lambda state: self.measurement_uv.set_saving(state == Qt.Checked))
        self.qt.saving_als_checkBox.stateChanged.connect(lambda state: self.measurement_ambient.set_saving(state == Qt.Checked))

        self.qt.lcd_site_num.display(self.site_number)
        self.qt.pushButton_change_site_inc.clicked.connect(self.increment_site_number)
        self.qt.pushButton_change_site_dec.clicked.connect(self.decrement_site_number)

        # Notes
        self.qt.textEdit_notes.textChanged.connect(lambda: self.temp_save_notes(self.site_number, self.qt.textEdit_notes.toPlainText()))
        self.qt.textEdit_notes.clear() # Ensure it is empty at the start

        # Calibrate UV
        self.qt.uv_calibrate_button.clicked.connect(self.do_calibrate_uv)

        # Kill Switch
        self.qt.pushButton_kill_switch.clicked.connect(lambda: self.pub_kill_switch.publish(Empty()))

        # Only connecting to temperature service since moisture requests the same route
        self.qt.query_analog_checkBox.stateChanged.connect(lambda state: self.measurement_temperature.set_enabled(state == Qt.Checked))
        self.qt.query_analog_rateMs_lineEdit.editingFinished.connect(
            lambda: self.measurement_temperature.change_rate(self.qt.query_analog_rateMs_lineEdit.text())
        )
        self.qt.query_analog_rateMs_lineEdit.setText(str(self.measurement_temperature.timer.period_ms))  # Set initial value

        # Only connecting to uv_index service since ambient requests the same route
        self.qt.query_ltr_checkBox.stateChanged.connect(lambda state: self.measurement_uv.set_enabled(state == Qt.Checked))
        self.qt.query_ltr_rateMs_lineEdit.editingFinished.connect(
            lambda: self.measurement_uv.change_rate(self.qt.query_ltr_rateMs_lineEdit.text())
        )
        self.qt.query_ltr_rateMs_lineEdit.setText(str(self.measurement_uv.timer.period_ms))  # Set initial value

        # Save to notes buttons
        self.qt.copy_attitude_pushButton.clicked.connect(self.copy_attitude_to_notes)
        self.qt.copy_notes_temp_pushButton.clicked.connect(
            lambda: self.copy_measurement_service_to_notes(
                self.measurement_temperature, units='°C', decimals=2))
        self.qt.copy_notes_moisture_pushButton.clicked.connect(
            lambda: self.copy_measurement_service_to_notes(
                self.measurement_moisture, units=' μS/cm', decimals=3))
        self.qt.copy_notes_uv_pushButton.clicked.connect(
            lambda: self.copy_measurement_service_to_notes(
                self.measurement_uv, title='UV Index', decimals=1))
        self.qt.copy_notes_ambient_pushButton.clicked.connect(
            lambda: self.copy_measurement_service_to_notes(
                self.measurement_ambient, units=' lux', decimals=1))

    def do_calibrate_uv(self):
        uv_index, ok = QtWidgets.QInputDialog.getDouble(
            self.qt, 
            "UV Index Calibration", 
            "Enter the current UV Index:", 
            decimals=2, 
            min=0.0, 
            max=15.0
        )
        if ok:
            self.pub_calibrate_uv.publish(Float32(data=uv_index))

    def temp_save_notes(self, site_number, text):
        """
        Saves the current notes under the given site.
        """
        self.site_notes[site_number] = {
            'text' : text,
            'saved' : False
        }

    def load_notes(self, site_number):
        if site_number in self.site_notes:
            self.qt.textEdit_notes.setPlainText(self.site_notes[site_number]['text'])
        else:
            self.qt.textEdit_notes.clear()

    def fetch_spectro_data(self):
        msg = Empty()
        self.pub_get_spectro.publish(msg)

    def fetch_uv_data(self):
        self.pub_get_uv.publish(Empty())

    def fetch_analog_sensors_data(self):
        self.pub_get_analog_sensors.publish(Bool( data=self.should_use_module_eeprom_calibration() ))

    def update_analog_sensor_values(self, msg: ScienceSensorValues):
        self.measurement_moisture.receive(msg.moisture)
        self.measurement_temperature.receive(msg.temperature)

    def update_ltr_sensor_values(self, msg: ScienceUvData):
        self.measurement_uv.receive(msg.uv)
        self.measurement_ambient.receive(msg.als)

    def update_spectro_data(self, msg):
        vals = msg.values
        self.spectro_table.updateData(vals)

    def save_spectro_values(self):
        """
        Saves the spectro data to a file.
        """
        
        # Publish the spectro data as a ScienceSaveSpectro message
        spectro_msg = ScienceSaveSpectro()
        spectro_msg.site = self.site_number
        spectro_msg.channels = [float(value) for value in self.spectro_table.values]
        self.pub_save_spectro.publish(spectro_msg)

    def toggle_sensor_save(self, measurement):
        """
        Called when any sensor radio button is called (moist, temp, fad)
        Tells science data saver to start saving values or to finish saving values.
        """

        print('Toggling Saving Sensor', measurement)
        self.sensor_saving[measurement] = not self.sensor_saving[measurement]
        if (measurement == 'moisture'): # Moisture radio button
            sensor_message = ScienceSaveSensor(
                site=self.site_number,
                measurement=measurement,
                observed_value=float(self.qt.lineEdit_moisture.text()),
                save=self.sensor_saving[p]
            )
        elif (measurement == 'temperature'): # Temp radio button
            sensor_message = ScienceSaveSensor(site=self.site_number, position=p, observed_value=float(self.qt.lineEdit_temperature.text()), save=self.sensor_saving[p])
        else:  # fad radio button, probably going to end up being useless.
            if self.qt.fad_radio.isChecked():
                self.fad_timer = self.create_timer(self.save_interval, self.stop_fad_saver)
                print("made timer for FAD")
            if (self.qt.lineEdit_fad.text() != ''):
                sensor_message = ScienceSaveSensor(site=self.site_number, position=p, observed_value=float(self.qt.lineEdit_fad.text()), save=self.sensor_saving[p])
            else:
                sensor_message = ScienceSaveSensor(site=self.site_number, position=p, observed_value=3.141592, save=self.sensor_saving[p])

        self.signals.sensor_save_signal.emit(sensor_message)

    def increment_site_number(self):
        """
        Increments the site number
        """
        self.site_number += 1
        self.qt.lcd_site_num.display(self.site_number)
        self.load_notes(self.site_number)

    def decrement_site_number(self):
        """
        Deccrements the site number
        """
        self.site_number -= 1
        self.site_number = max(1, self.site_number)  # Ensure site number doesn't go below 1
        self.qt.lcd_site_num.display(self.site_number)
        self.load_notes(self.site_number)

    def save_notes(self):
        """
        Saves the current notes under the given site.
        """
        for site in self.site_notes.keys():
            if self.site_notes[site]['saved'] == False:
                self.site_notes[site]['saved'] = True
                self.save_notes_msg = ScienceSaveNotes(
                    site = site,
                    notes = self.site_notes[site]['text']
                )
                self.signals.notes_save_signal.emit(self.save_notes_msg)
        self.get_logger().info('Saved notes.')

    def graph_sensor_values(self, position):
        manual_points = []
        analog_vals = []
        coefficients_path = ""
        coefficients_file = ""

        match(position):
            case 0:
                file_name = "moisture-plot-1.txt"
                coefficients_file = "moisture_coefficients.txt"
            case 1:
                file_name = "temperature-plot-1.txt"
                coefficients_file = "temperature_coefficients.txt"
            case 2:
                file_name = "fad-plot-1.txt"
                coefficients_file = "fad_coefficients.txt"
            case _: #Wildcard, acts like else
                print("Err: this sensor does not have data to graph")
                return
        file_path = os.path.join(self.science_data_path, file_name)
        coefficients_path = os.path.join(self.science_data_path, coefficients_file)

        #Check file existence
        if not os.path.exists(file_path):
            print("Err: file does not exist")
            return

        with open(file_path, 'r') as f:
            for line in f:
                split = line.split()
                manual_points.append(float(split[0]))
                reading_series =[]
                for i in split[1:]:
                    reading_series.append(float(i))
                    #Normalize the values form zero to 1.
                    reading_series[-1] = reading_series[-1]/1023
                analog_vals.append(reading_series)
            
            #Show an updated graph with the new point
            dummy_manuals = []
            for i in range(len(analog_vals)):
                for j in analog_vals[i]:
                    dummy_manuals.append(manual_points[i])
            dummy_analog = []
            for i in analog_vals:
                for j in i:
                    dummy_analog.append(j)

            plt.scatter(dummy_analog,dummy_manuals)
            plt.pause(0.5)
        #Add something so you can decide what order polynomial you want.
        order = int(input("What order polynomial do you want to fit? [0 - 6]\n"))
        P0 = np.zeros((1,6-order))

        #Plot the points alongside the polyfit.
        analog_vals = np.array(dummy_analog)
        manual_points = np.array(dummy_manuals)
        P1 = np.polyfit(analog_vals, manual_points, order)
        P = np.concatenate((P0,P1),axis=None)
        x = np.linspace(0,1,500)
        poly_y = P[0]*x**6+P[1]*x**5+P[2]*x**4+P[3]*x**3+P[4]*x**2+P[5]*x + P[6]
        plt.figure()
        plt.scatter(analog_vals, manual_points, label="input data")
        plt.xlabel("Arduino Digital Readout")
        plt.ylabel("Reference Temperature (deg C)")
        plt.plot(x, poly_y, label="polynomial fit")
        plt.legend()
        plt.show()

        with open(coefficients_path, 'w') as f:
            line = ''
            for i in P:
                line += str(i)
                line += " "
            f.write(line)

    def estimate_reading(self, position):
        match(position):
            case 0:
                coefficients_file = "moisture_coefficients.txt"
                try:
                    val = self.qt.lineEdit_moisture_2.text()
                    x = float(val)/1023
                except (ValueError):
                    self.get_logger().error(f"{val} is not a valid number")
                    return
            case 1:
                coefficients_file = "temperature_coefficients.txt"
                try:
                    val = self.qt.lineEdit_temperature_2.text()
                    x = float(val)/1023
                except (ValueError):
                    self.get_logger().error(f"{val} is not a valid number")
                    return
            case 2:
                coefficients_file = "fad_coefficients.txt"
                try:
                    val = self.qt.lineEdit_fad.text()
                    x = float(val)/1023
                except (ValueError):
                    self.get_logger().error(f"{val} is not a valid number")
                    return
            case _: #Wildcard, acts like else. This should never happen.
                print("Err: this sensor does not have data to graph")
                return
        coefficients_path = os.path.join(self.science_data_path, coefficients_file)
        # self.get_logger().info([(f) for f in os.listdir(self.science_data_path) if os.path.isfile(os.path.join(self.science_data_path, f))])

        
        with open(coefficients_path, 'r') as f:
            coefs = f.read().split()
            result = 0
            for i in range(len(coefs)):
                result += (float(coefs[i]) * (float(x)**i))
        sensor = "None"
        if position == 0: sensor = "Moisture"
        elif position == 1: sensor = "Temperature"
        elif position == 2: sensor = "FAD"
        else: sensor = "Unknown sensor"
        self.qt.textBrowser.setPlainText(sensor + f" reading is {result}")
        
    def update_fad_intensity_value(self, msg):
        #Insert code to send reading to save?
        # print('Displaying intensity!', msg)
        self.qt.le_fad.setText(str(msg.intensity_avg))

    def update_pos_vel_time(self, msg):
        altitude = f'{msg.gps.altitude} ft'
        heading = f'{msg.map_yaw}'
        coordinates = f'{msg.gps.latitude}, {msg.gps.longitude}'
        # print(f'(lat, long, altitude, heading): ({coordinates}, {altitude}, {heading})')

        self.qt.lbl_altitude.setText(altitude)
        self.qt.lbl_heading.setText(heading)
        self.qt.lbl_coordinates.setText(coordinates)

    def update_current_tool(self, msg):
        if msg.data == True: # Using Probe
            self.qt.current_tool_label.setText("PROBE")
        elif msg.data == False: # Not using probe
            self.qt.current_tool_label.setText("AUGER")
        else:
            self.qt.current_tool_label.setText("NONE")

    def fad_detector_get_point(self, event=None): #Written by chat
        print('Calibrate FAD')
        if not self.cli.service_is_ready():
            self.get_logger().error("Camera control service is not available. Try again in a bit")
            return

        self.req = CameraControl.Request()
        self.req.camera.client_address = "{}@{}".format(os.getlogin(), self.base_ip)
        self.req.camera.camera_name = 'fadCam'
        self.req.site_name = 'fad_calibration'
        self.req.calibrate = True

        # Call the service asynchronously
        self.future = self.cli.call_async(self.req)
        self.future.add_done_callback(self.handle_fad_response)

    def handle_fad_response(self, future): #written by chat
        try:
            response = future.result()
            self.get_logger().info(f"FAD Intensity: {response.intensity}")
            self.update_fad_intensity_value(response)
        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")

    def get_base_ip(self):
        ip = os.getenv("BASE_ADDRESS")
        if ip is None:
            ip = "192.168.1.65"
        return ip
    
    def append_to_notes(self, text):
        """
        Appends the given text to the current notes.
        """
        current_notes = self.qt.textEdit_notes.toPlainText()
        if current_notes == "":
            updated_notes = text
        elif current_notes[-1] == '\n':
            updated_notes = current_notes + text
        else:
            updated_notes = current_notes + "\n" + text
        
        # Push the new text to the text edit widget
        self.qt.textEdit_notes.setPlainText(updated_notes)

        # Save the updated notes temporarily
        self.temp_save_notes(self.site_number, updated_notes)

    def copy_attitude_to_notes(self):
        """
        Appends the current altitude, heading, and coordinates to the site notes.
        """
        altitude = self.qt.lbl_altitude.text()
        heading = self.qt.lbl_heading.text()
        coordinates = self.qt.lbl_coordinates.text()
        note_entry = f"Rover Attitude:\n Altitude: {altitude}\n Heading: {heading}\n Coordinates: {coordinates}\n"
        self.append_to_notes(note_entry)

    def copy_measurement_service_to_notes(self, meas_service, title="", units="", decimals=None):
        """
        Appends the current altitude, heading, and coordinates to the site notes.
        """
        recall = meas_service.recall()
        data = 0.0 if recall is None else recall
        value = f"{data:.{decimals}f}" if decimals is not None else str(data)
        title = ' '.join(word.capitalize() for word in meas_service.measurement_name.split('_')) if title == "" else title
        note_entry = f"{title}: {value}{units}\n"
        self.append_to_notes(note_entry)

class MeasurementService:
    def __init__(self, node, measurement_name, period_ms, enabled, saving, request_func, save_func):
        self.node = node
        self.measurement_name = measurement_name
        self.saving = saving
        self.timer = Timer(node, period_ms, enabled, request_func)
        self.save_func = save_func
        self.data = None

    def change_rate(self, new_rate):
        try:
            new_rate = int(new_rate)
            if new_rate > 0:
                self.timer.change_rate(new_rate)
                self.node.get_logger().info(f"Query rate for {self.measurement_name} updated to {new_rate} ms.")
            else:
                self.node.get_logger().warning("Query rate must be a positive integer.")
        except ValueError:
            self.node.get_logger().warning("Invalid input for query rate. Please enter a valid integer.")

    def set_enabled(self, enabled):
        self.timer.enable() if enabled else self.timer.disable()

    def set_saving(self, saving):
        self.saving = saving

    def enable(self):
        self.timer.enable()

    def disable(self):
        self.timer.enable()

    def receive(self, data):
        # Should be call by children classes
        self.data = data
        if self.saving:
            self.save_func(data)

    def recall(self):
        return self.data

class SensorMeasurementService(MeasurementService):
    def __init__(self, node, measurement_name, period_ms, enabled, saving, request_func, lcd_display=None):
        super().__init__(node, measurement_name, period_ms, enabled, saving, request_func,
            lambda data: node.pub_save_sensor.publish(
                ScienceSaveSensor(
                    site=node.site_number,
                    timestamp=time.time(),
                    measurement=measurement_name,
                    observed_value=float(data)
                )
            )
        )
        self.lcd_display = lcd_display

    def receive(self, data):
        self.lcd_display.display(data)
        super().receive(data)

class TemperatureMS(SensorMeasurementService):
    def __init__(self, node, period_ms, enabled, saving):
        super().__init__(node, "temperature", period_ms, enabled, saving,
            lambda: node.fetch_analog_sensors_data(),
            node.qt.lcd_temp
        )

class MoistureMS(SensorMeasurementService):
    def __init__(self, node, period_ms, enabled, saving):
        super().__init__(node, "moisture", period_ms, enabled, saving,
            lambda: node.fetch_analog_sensors_data(),
            node.qt.lcd_moist
        )

class UVIndexMS(SensorMeasurementService):
    def __init__(self, node, period_ms, enabled, saving):
        super().__init__(node, "uv_index", period_ms, enabled, saving,
            lambda: node.fetch_uv_data(),
            node.qt.lcd_uv
        )

class AmbientLightIndexMS(SensorMeasurementService):
    def __init__(self, node, period_ms, enabled, saving):
        super().__init__(node, "ambient_light", period_ms, enabled, saving,
            lambda: node.fetch_uv_data(),
            node.qt.lcd_ambient
        )

    
class Timer:
    def __init__(self, node, period_ms, enabled, func):
        self.node = node
        self.period_ms = period_ms
        self.timer = None
        self.func = func
        self.enabled = enabled
        if self.enabled:
            self.build()

    def end_timer(self):
        if self.timer is not None:
            self.timer.destroy()

    def build(self):
        self.end_timer()
        self.timer = self.node.create_timer((self.period_ms / 1000.0), self.func)
        # self.node.get_logger().info(f"Timer created with period: {self.period_ms} ms")

    def change_rate(self, new_period):
        if new_period != self.period_ms:
            self.period_ms = new_period
        if self.enabled:
            self.build() # Update the timer

    def enable(self):
        self.enabled = True
        self.build()

    def disable(self):
        self.enabled = False
        self.end_timer()
    
def main(args=None):
    rclpy.init(args=args)
    
    # Create the Qt application
    app = QtWidgets.QApplication(sys.argv)
    gui = science_GUI()  # Initialize your GUI

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