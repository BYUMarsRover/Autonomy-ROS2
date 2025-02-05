#!/usr/bin/python3

from PyQt5 import QtWidgets, uic
from python_qt_binding.QtCore import QObject, Signal
import rclpy
from rover_msgs.srv import CameraControl
from rover_msgs.msg import ScienceToolPosition, ScienceSensorValues, ScienceSaveSensor, ScienceSaveNotes, ScienceFADIntensity, Camera, RoverStateSingleton
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import matplotlib.pyplot as plt
import numpy as np

import os
import sys

class Signals(QObject):
    sensor_signal = Signal(ScienceSensorValues)
    auger_position = Signal(ScienceToolPosition)
    sensor_save_signal = Signal(ScienceSaveSensor)
    notes_save_signal = Signal(ScienceSaveNotes)
    fad_intensity_signal = Signal(ScienceFADIntensity)

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
        # uic.loadUi(os.path.expanduser('~') + '/mars_ws/src/science/science/gui/science_GUI.ui', self.qt) # Load the .ui file
        self.qt.show() # Show the GUI

        self.base_ip = self.get_base_ip()
        self.cli = self.create_client(CameraControl, 'camera_control')
        if not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Camera control not available, waiting...')
        self.req = CameraControl.Request()
        self.future = self.cli.call_async(self.req)

        self.temperature = 1
        self.moisture = 0
        self.fad = 2

        self.fad_calibration_interval = 2
        self.site_number = 1
        self.initialize_timers()
        self.task_launcher_init()
        self.sensor_saving = [False] * 3  # temp, moisture, fad
        self.temperature_coefficients = [[],[],[],[],[],[]]
        self.moisture_coefficients = [[],[],[],[],[],[]]

        self.science_data_path = os.path.expanduser("~/science_data/site-1")
        # file_path = os.path.join(science_data_path, file_name)

        if self.future.result() is not None:
            self.get_logger().info(f"{self.future.result()}")
        else:
            self.get_logger().error(f"Service call failed {self.future.exception()}")

    def initialize_timers(self):
        self.save_interval = 10
        self.moisture_timer = self.create_timer(self.save_interval, self.stop_moist_saver)
        self.temp_timer = self.create_timer(self.save_interval, self.stop_temp_saver)
        self.fad_timer = self.create_timer(self.save_interval, self.stop_fad_saver)

    def task_launcher_init(self):
        self.signals = Signals()

        #Read in coefficients. TODO - implement throw error if not defined.
        # file_path = os.path.join(self.science_data_path, "polynomials.txt")
        # with open(file_path, 'r') as f:
        #     moisture_values = f[0].split()
        #     for i in range(len(moisture_values)):
        #         self.moisture_coefficients[i] = moisture_values[i]
        #     temp_values = f[1].split()
        #     for i in range(len(temp_values)):
        #         self.temperature_coefficients[i] = temp_values[i]

        self.qt.pushButton_save_notes.clicked.connect(self.save_notes)
        self.qt.pushButton_fad.clicked.connect(self.fad_detector_calibration)

        self.qt.pushButton_temperature.clicked.connect(lambda: self.graph_sensor_values(1))
        self.qt.pushButton_moisture.clicked.connect(lambda: self.graph_sensor_values(0))

        self.qt.moist_radio.toggled.connect(lambda: self.toggle_sensor_save(0))  # moist
        self.qt.temp_radio.toggled.connect(lambda: self.toggle_sensor_save(1))  # temp
        self.qt.fad_radio.toggled.connect(lambda: self.toggle_sensor_save(2))  # fad

        self.qt.lcd_site_num.display(self.site_number)
        self.qt.pushButton_change_site.clicked.connect(self.increment_site_number)

        self.pub_save_sensor = self.create_publisher(ScienceSaveSensor, '/science_save_sensor', 1) #figure this out
        self.pub_save_notes = self.create_publisher(ScienceSaveNotes, '/science_save_notes', 1)

        self.signals.sensor_signal.connect(self.update_sensor_values)
        self.signals.auger_position.connect(self.update_auger_position)
        self.signals.sensor_save_signal.connect(self.pub_save_sensor.publish)
        self.signals.notes_save_signal.connect(self.pub_save_notes.publish)
        self.signals.fad_intensity_signal.connect(self.update_fad_intensity_value)

        self.science_sensor_values = self.create_subscription(ScienceSensorValues, '/science_sensor_values', self.signals.sensor_signal.emit, 10)
        self.science_auger_position = self.create_subscription(ScienceToolPosition, '/science_auger_position', self.signals.auger_position.emit, 10)
        self.science_fad_calibration = self.create_subscription(ScienceFADIntensity, '/science_fad_calibration', self.signals.fad_intensity_signal.emit, 10)
        self.rover_state_singleton = self.create_subscription(RoverStateSingleton, '/odometry/rover_state_singleton', self.update_pos_vel_time, 10)

    def toggle_sensor_save(self, p):
        """
        Called when any sensor radio button is called (moist, temp, fad)
        Tells science data saver to start saving values or to finish saving values.
        p: sensor number, [0: moist, 1: temp, 2: fad]
        """

        print('Toggling Saving Sensor', p)
        self.sensor_saving[p] = not self.sensor_saving[p]
        if (p == 0):
            self.sensor_message = ScienceSaveSensor(site=self.site_number, position=p, observed_value=float(self.qt.lineEdit_moisture.text()), save=self.sensor_saving[p])
        elif (p == 1):
            self.sensor_message = ScienceSaveSensor(site=self.site_number, position=p, observed_value=float(self.qt.lineEdit_temperature.text()), save=self.sensor_saving[p])
        self.signals.sensor_save_signal.emit(self.sensor_message)

    def stop_temp_saver(self):
        self.temp_timer.cancel()
        self.qt.temp_radio.setChecked(False)

    def stop_moist_saver(self):
        self.moisture_timer.cancel()
        self.qt.moist_radio.setChecked(False)

    def stop_fad_saver(self):
        self.fad_timer.cancel()
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
                file_path = os.path.join(self.science_data_path, file_name)
                coefficients_file = "moisture_coefficients.txt"
                coefficients_path = os.path.join(self.science_data_path, coefficients_file)
            case 1:
                file_name = "temperature-plot-1.txt"
                file_path = os.path.join(self.science_data_path, file_name)
                coefficients_file = "moisture_coefficients.txt"
                coefficients_path = os.path.join(self.science_data_path, coefficients_file)
            case _: #Wildcard, acts like else
                print("Err: this sensor does not have data to graph")
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
            # plt.draw()
            plt.pause(0.5)
            # Have it ask you to save the point or delete after showing you an updated graph.
            # keep = input("Do you want to keep this point? [y]/n\n")
            keep = "y"
            if not (keep == "y" or keep =="Y" or keep == "[Y]" or keep == "[y]" or keep == ""):
                manual_points.pop()
                analog_vals.pop()
                plt.cla()
                plt.scatter(dummy_analog,dummy_manuals)
                plt.xlabel("Arduino Digital Readout")
                plt.ylabel("Reference Temperature (deg C)")
                plt.pause(0.5)
        #Add something so you can decide what order polynomial you want.
        order = int(input("What order polynomial do you want to fit? [0 - 6]\n"))
        # order = 1
        P0 = np.zeros((1,6-order))
        #Plot the points alongside the polyfit.
        analog_vals = np.array(dummy_analog)
        manual_points = np.array(dummy_manuals)
        P1 = np.polyfit(analog_vals, manual_points, order)
        P = np.concatenate((P0,P1),axis=None)
        x = np.linspace(0,1,500)
        poly_y = P[0]*x**6+P[1]*x**5+P[2]*x**4+P[3]*x**3+P[4]*x**2+P[5]*x + P[6]
        # print(P)
        plt.figure()
        plt.scatter(analog_vals, manual_points, label="input data")
        plt.xlabel("Arduino Digital Readout")
        plt.ylabel("Reference Temperature (deg C)")
        plt.plot(x, poly_y, label="polynomial fit")
        plt.legend()
        plt.show()
        print(P)
        # with open()

        
    def update_fad_intensity_value(self, msg):
        print('Displaying intensity!', msg)
        self.qt.le_fad.setPlaceholderText(str(msg))

    def update_pos_vel_time(self, msg):
        altitude = f'{msg.gps.altitude} ft'
        heading = f'{msg.map_yaw}'
        coordinates = f'{msg.gps.latitude}, {msg.gps.longitude}'

        print(f'(lat, long, altitude, heading): ({coordinates}, {altitude}, {heading})')

        self.qt.lbl_altitude.setText(altitude)
        self.qt.lbl_heading.setText(heading)
        self.qt.lbl_coordinates.setText(coordinates)

    def update_auger_position(self, msg):
        """
        Updates the augur display.

        This is like this because the photoresistors are backwards on the board.
        """
        if msg.position == 0:
            self.qt.lcd_auger.display(2)
        elif msg.position == 1:
            self.qt.lcd_auger.display(1)
        else:
            self.qt.lcd_auger.display(-1)

    def fad_detector_calibration(self, event=None):
        print('Calibrate FADD')
        site_name = 'fad_calibration'
        camera = Camera()
        camera.client_address = "{}@{}".format(os.getlogin(), self.base_ip)
        camera.camera_name = 'fadCam'
        response = self.camera_control(camera=camera, site_name=site_name, calibrate=True)

        self.update_fad_intensity_value(response.intensity)

    def get_base_ip(self):
        ip = os.getenv("BASE_ADDRESS")
        if ip is None:
            ip = "192.168.1.65"
        return ip
    
def main(args=None):
    rclpy.init(args=args)
    app = QtWidgets.QApplication(sys.argv)
    window = science_GUI()

    window.qt.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()