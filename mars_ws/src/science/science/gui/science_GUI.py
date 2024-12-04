#!/usr/bin/python3

from PyQt5 import QtWidgets, uic
from python_qt_binding.QtCore import QObject, Signal
import rclpy
from rover_msgs.srv import CameraControl
from rover_msgs.msg import ScienceToolPosition, ScienceSensorValues, ScienceSaveSensor, ScienceSaveNotes, ScienceFADIntensity, Camera, RoverStateSingleton
from rclpy.node import Node

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

        # rospy.init_node("science_GUI")
        super().__init__('science_GUI')
        self.qt = QtWidgets.QWidget()
        uic.loadUi(os.path.expanduser('~') + '/mars_ws/src/science/science/gui/science_GUI.ui', self.qt) # Load the .ui file
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
        if self.future.result() is not None:
            print(self.future.result())
        else:
            print('Service call failed %r' % (self.future.exception(),))

    def initialize_timers(self):
        self.save_interval = 10
        self.moisture_timer = self.create_timer(self.save_interval, self.stop_moist_saver)
        self.temp_timer = self.create_timer(self.save_interval, self.stop_temp_saver)
        self.fad_timer = self.create_timer(self.save_interval, self.stop_fad_saver)

    def task_launcher_init(self):
        self.signals = Signals()

        self.qt.pushButton_save_notes.clicked.connect(self.save_notes)
        self.qt.pushButton_fad.clicked.connect(self.fad_detector_calibration)

        self.qt.moist_radio.toggled.connect(lambda: self.toggle_sensor_save(0))  # moist
        self.qt.temp_radio.toggled.connect(lambda: self.toggle_sensor_save(1))  # temp
        self.qt.fad_radio.toggled.connect(lambda: self.toggle_sensor_save(2))  # fad

        self.qt.lcd_site_num.display(self.site_number)
        self.qt.pushButton_change_site.clicked.connect(self.increment_site_number)

        self.pub_save_sensor = self.create_publisher(ScienceSaveSensor, '/science_save_sensor', 1)
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
        self.signals.sensor_save_signal.emit(ScienceSaveSensor(self.site_number, p, self.sensor_saving[p]))

        if p == self.temperature:
            if self.temp_radio.isChecked():
                self.temp_timer.start()
        elif p == self.moisture:
            if self.moist_radio.isChecked():
                self.moisture_timer.start()
        else:  # fad radio button
            if self.fad_radio.isChecked():
                self.fad_timer.start()

    def stop_temp_saver(self):
        self.temp_timer.cancel()
        self.temp_radio.setChecked(False)

    def stop_moist_saver(self):
        self.moisture_timer.cancel()
        self.moist_radio.setChecked(False)

    def stop_fad_saver(self):
        self.fad_timer.cancel()
        self.fad_radio.setChecked(False)

    def increment_site_number(self):
        """
        Increments the site number
        """
        self.site_number += 1
        self.lcd_site_num.display(self.site_number)

    def save_notes(self):
        """
        Saves the current notes under the given site.
        """
        print('Saving notes.')
        self.signals.notes_save_signal.emit(ScienceSaveNotes(int(self.textEdit_site.toPlainText()), self.textEdit_notes.toPlainText()))
        print('Notes sent.')

    def update_sensor_values(self, msg):
        temperature = msg.temperature
        moisture = msg.moisture

        self.qt.lcd_moist.display(moisture)
        self.qt.lcd_temp.display(temperature)

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