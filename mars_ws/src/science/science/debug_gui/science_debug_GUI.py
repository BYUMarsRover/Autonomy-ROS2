#!/usr/bin/python3

from PyQt5 import QtWidgets, uic
from python_qt_binding.QtCore import QObject, Signal
import rclpy
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

class science_debug_GUI(Node):
    def __init__(self):
        
        super().__init__('science_debug_GUI')
        self.qt = QtWidgets.QWidget()

        debug_skeleton_ui_path = os.path.join(
            get_package_share_directory('science'),
            'debug_gui',
            'skeleton.ui'
            )
        
        command_widget_path = os.path.join(
            get_package_share_directory('science'),
            'debug_gui',
            'command_widget.ui'
            )

        uic.loadUi(debug_skeleton_ui_path, self.qt)

        # TODO Load in the command widget and dynmically insert them into the skeleton

        self.qt.show() # Show the GUI

        self.base_ip = self.get_base_ip()

        self.task_launcher_init()
        

    def task_launcher_init(self):
        pass
        # self.signals = Signals()

        # self.qt.pushButton_save_notes.clicked.connect(self.save_notes)
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
    
    def get_base_ip(self):
        ip = os.getenv("BASE_ADDRESS")
        if ip is None:
            ip = "192.168.1.65"
        return ip
    
def main(args=None):
    rclpy.init(args=args)
    
    # Create the Qt application
    app = QtWidgets.QApplication(sys.argv)
    gui = science_debug_GUI()  # Initialize your GUI

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