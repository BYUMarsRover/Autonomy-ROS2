#!/usr/bin/python3

from PyQt5 import QtWidgets, uic
from PyQt5.QtCore import QObject, pyqtSignal, QTimer, Qt, QAbstractTableModel
from PyQt5.QtWidgets import QApplication, QTableView
from python_qt_binding.QtCore import QObject, Signal
import rclpy
from std_msgs.msg import Empty
from rover_msgs.srv import CameraControl
from rover_msgs.msg import ScienceSensorValues, ScienceSaveSensor, ScienceSaveNotes, ScienceSaveFAD, ScienceFADIntensity, Camera, RoverStateSingleton
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import matplotlib.pyplot as plt
import numpy as np

import os
import sys

class Signals(QObject):
    sensor_signal = Signal(ScienceSensorValues)
    # auger_position = Signal(ScienceToolPosition)
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
        self.qt = QtWidgets.QWidget()

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
        wavelengths = [400 + 20 * i for i in range(18)]
        values = [0 for i in range(18)]

        # Set Model to QTableView
        self.spectro_table = WavelengthTableModel(wavelengths, values)
        self.qt.tableView.setModel(self.spectro_table)

        self.temperature = 1
        self.moisture = 0
        self.fad = 2

        self.save_interval = 10

        self.fad_calibration_interval = 2
        self.site_number = 1
        self.initialize_timers()
        self.task_launcher_init()
        self.sensor_saving = [False] * 3  # temp, moisture, fad
        self.temperature_coefficients = [[],[],[],[],[],[]]
        self.moisture_coefficients = [[],[],[],[],[],[]]

        self.science_data_path = os.path.expanduser("~/science_data/site-1")

        # Read in coefficients. 
        moisture_path = os.path.join(self.science_data_path, "moisture_polynomials.txt")
        temp_path = os.path.join(self.science_data_path, "temp_polynomials.txt")
        if os.path.exists(moisture_path):
            with open(moisture_path, 'r') as f:
                moisture_values = f[0].split()
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

        # if self.future.result() is not None:
        #     self.get_logger().info(f"{self.future.result()}")
        # else:
        #     self.get_logger().error(f"Service call failed {self.future.exception()}")

    def initialize_timers(self):
        self.moisture_timer = self.create_timer(self.save_interval, self.stop_moist_saver)#These are probably redundant in ros2
        self.temp_timer = self.create_timer(self.save_interval, self.stop_temp_saver)
        

    def task_launcher_init(self):
        self.signals = Signals()

        self.qt.pushButton_save_notes.clicked.connect(self.save_notes)
        self.qt.pushButton_fad.clicked.connect(self.fad_detector_get_point)
        self.qt.pushButton_fad_calibration.clicked.connect(self.save_fad)

        self.qt.pushButton_moisture.clicked.connect(lambda: self.graph_sensor_values(0))
        self.qt.pushButton_temperature.clicked.connect(lambda: self.graph_sensor_values(1))
        self.qt.pushButton_fad_graph.clicked.connect(lambda: self.graph_sensor_values(2))

        self.qt.pushButton_moisture_2.clicked.connect(lambda: self.estimate_reading(0))
        self.qt.pushButton_temperature_2.clicked.connect(lambda: self.estimate_reading(1))
        self.qt.pushButton_fad_estimate.clicked.connect(lambda: self.estimate_reading(2))

        #TODO
        # self.qt.pushButton_spectro.clicked.connect(lambda: self.fetch_spectro_data())
        # self.qt.pushButton_uv.clicked.connect(lambda: self.fetch_uv_data())
        self.qt.pushButton_spectro.clicked.connect(lambda: self.pub_get_spectro.publish())
        self.qt.pushButton_uv.clicked.connect(lambda: self.pub_get_uv.publish())

        self.qt.moist_radio.toggled.connect(lambda: self.toggle_sensor_save(0))  # moist
        self.qt.temp_radio.toggled.connect(lambda: self.toggle_sensor_save(1))  # temp
        self.qt.fad_radio.toggled.connect(lambda: self.toggle_sensor_save(2))  # fad

        self.qt.lcd_site_num.display(self.site_number)
        self.qt.pushButton_change_site.clicked.connect(self.increment_site_number)

        self.pub_save_sensor = self.create_publisher(ScienceSaveSensor, '/science_save_sensor', 1) #figure this out
        self.pub_save_notes = self.create_publisher(ScienceSaveNotes, '/science_save_notes', 1)
        self.pub_save_fad = self.create_publisher(ScienceSaveFAD, '/science_save_fad', 1)
        
        #TODO - make new messages and integrate with science serial interface
        self.pub_get_spectro = self.create_publisher(Empty, '/science_spectro_request', 1)
        self.pub_get_uv = self.create_publisher(Empty, '/science_uv_request', 1)

        self.signals.sensor_signal.connect(self.update_sensor_values)
        # self.signals.auger_position.connect(self.update_auger_position)
        self.signals.sensor_save_signal.connect(self.pub_save_sensor.publish)
        self.signals.FAD_save_signal.connect(self.pub_save_fad.publish)
        self.signals.notes_save_signal.connect(self.pub_save_notes.publish)
        self.signals.fad_intensity_signal.connect(self.update_fad_intensity_value)

        self.science_sensor_values = self.create_subscription(ScienceSensorValues, '/science_sensor_values', self.signals.sensor_signal.emit, 10)
        # self.science_auger_position = self.create_subscription(ScienceToolPosition, '/science_auger_position', self.signals.auger_position.emit, 10)
        self.science_fad_calibration = self.create_subscription(ScienceFADIntensity, '/science_fad_calibration', self.signals.fad_intensity_signal.emit, 10)
        self.rover_state_singleton = self.create_subscription(RoverStateSingleton, '/odometry/rover_state_singleton', self.update_pos_vel_time, 10)
        self.sub_spectro = self.create_subscription(ScienceSpectroData, '/science_spectro_data', self.update_spectro_data, 10)
        self.sub_uv = self.create_subscription(ScienceUvData, '/science_uv_data', self.update_uv_values, 10)

    # def fetch_spectro_data(self):
    #     self.pub_get_spectro.publish()

    def update_spectro_data(self, msg)
        vals = msg.values#TODO - get new message
        self.spectro_table.updateData(vals)

    def update_uv_values(self, msg):
        uv = msg.uv
        als = msg.als
        self.qt.lcd_uv.display(uv)
        self.qt.ambient.display(als)
    
    def toggle_sensor_save(self, p):
        """
        Called when any sensor radio button is called (moist, temp, fad)
        Tells science data saver to start saving values or to finish saving values.
        p: sensor number, [0: moist, 1: temp, 2: fad]
        """

        print('Toggling Saving Sensor', p)
        self.sensor_saving[p] = not self.sensor_saving[p]
        if (p == 0): # Moisture radio button
            sensor_message = ScienceSaveSensor(site=self.site_number, position=p, observed_value=float(self.qt.lineEdit_moisture.text()), save=self.sensor_saving[p])
        elif (p == 1): # Temp radio button
            sensor_message = ScienceSaveSensor(site=self.site_number, position=p, observed_value=float(self.qt.lineEdit_temperature.text()), save=self.sensor_saving[p])
        else:  # fad radio button, probably going to end up being useless.
            if self.qt.fad_radio.isChecked():
                self.fad_timer = self.create_timer(self.save_interval, self.stop_fad_saver)
                print("made timer for FAD")
            if (self.qt.lineEdit_fad.text() != ''):
                sensor_message = ScienceSaveSensor(site=self.site_number, position=p, observed_value=float(self.qt.lineEdit_fad.text()), save=self.sensor_saving[p])
            else:
                #Find how do get logger
                sensor_message = ScienceSaveSensor(site=self.site_number, position=p, observed_value=3.141592, save=self.sensor_saving[p])

        self.signals.sensor_save_signal.emit(sensor_message)

    def save_fad(self):
        self.sensor_saving[2] = True #2 is the FAD value, consider changing this to a const
        sensor_message = ScienceSaveFAD(site=self.site_number, intensity=int(self.qt.le_fad.text()), observed=float(self.qt.lineEdit_fad.text()))
        self.signals.FAD_save_signal.emit(sensor_message)

    def stop_temp_saver(self):
        self.temp_timer.cancel()
        self.qt.temp_radio.setChecked(False)

    def stop_moist_saver(self):
        self.moisture_timer.cancel()
        self.qt.moist_radio.setChecked(False)

    def stop_fad_saver(self):
        # self.fad_timer.cancel()
        print("Attempting to uncheck the FAD")
        self.fad_timer = None
        self.qt.fad_radio.setChecked(False)

    def increment_site_number(self):
        """
        Increments the site number
        """
        self.site_number += 1
        self.qt.lcd_site_num.display(self.site_number)

    def save_notes(self):
        """
        Saves the current notes under the given site.
        """
        print('Saving notes.')
        self.save_notes_msg = ScienceSaveNotes()
        self.signals.notes_save_signal.emit(self.save_notes_msg)
        print('Notes sent.')

    def update_sensor_values(self, msg):
        # print("updating LCD displays")
        temperature = msg.temperature
        moisture = msg.moisture

        self.qt.lcd_moist.display(moisture)
        self.qt.lcd_temp.display(temperature)

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
            # Have it ask you to save the point or delete after showing you an updated graph.
            # Need to decide if this is useful or not
            # keep = "y"
            # if not (keep == "y" or keep =="Y" or keep == "[Y]" or keep == "[y]" or keep == ""):
            #     manual_points.pop()
            #     analog_vals.pop()
            #     plt.cla()
            #     plt.scatter(dummy_analog,dummy_manuals)
            #     plt.xlabel("Arduino Digital Readout")
            #     plt.ylabel("Reference Temperature (deg C)")
            #     plt.pause(0.5)
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
                    val = self.qt.lineEdit_moisture_2.text()
                    x = float(val)/1023
                except (ValueError):
                    self.get_logger().error(f"{val} is not a valid number")
                    return
            case 2:
                coefficients_file = "fad_coefficients.txt"
                try:
                    val = self.qt.lineEdit_moisture_2.text()
                    x = float(val)/1023
                except (ValueError):
                    self.get_logger().error(f"{val} is not a valid number")
                    return
            case _: #Wildcard, acts like else. This should never happen.
                print("Err: this sensor does not have data to graph")
                return
        coefficients_path = os.path.join(self.science_data_path, coefficients_file)
        
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

        print(f'(lat, long, altitude, heading): ({coordinates}, {altitude}, {heading})')

        self.qt.lbl_altitude.setText(altitude)
        self.qt.lbl_heading.setText(heading)
        self.qt.lbl_coordinates.setText(coordinates)

    # def update_auger_position(self, msg):
    #     """
    #     Updates the augur display.

    #     This is like this because the photoresistors are backwards on the board.
    #     """
    #     if msg.position == 0:
    #         self.qt.lcd_auger.display(2)
    #     elif msg.position == 1:
    #         self.qt.lcd_auger.display(1)
    #     else:
    #         self.qt.lcd_auger.display(-1)

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


        # print('Calibrate FADD')
        # site_name = 'fad_calibration'
        # camera = Camera()
        # camera.client_address = "{}@{}".format(os.getlogin(), self.base_ip)
        # camera.camera_name = 'fadCam'
        # response = self.camera_control(camera=camera, site_name=site_name, calibrate=True)

        # self.update_fad_intensity_value(response.intensity)

    def get_base_ip(self):
        ip = os.getenv("BASE_ADDRESS")
        if ip is None:
            ip = "192.168.1.65"
        return ip
    
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