#!/usr/bin/python3

from PyQt5 import QtWidgets, uic, QtGui
from PyQt5.QtCore import Qt
import science.routine_gui.resources
from python_qt_binding.QtCore import QObject, Signal
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from science.function_mapping.function_map import ScienceModuleFunctionList as SMFL

import os
import sys

class Signals(QObject):
    science_rx_signal = Signal()
    science_tx_signal = Signal()
    # sensor_save_signal = Signal(ScienceSaveSensor)
    # FAD_save_signal = Signal(ScienceSaveFAD)
    # notes_save_signal = Signal(ScienceSaveNotes)
    # fad_intensity_signal = Signal(ScienceFADIntensity)

class science_routine_GUI(Node):
    def __init__(self):
        
        super().__init__('science_routine_GUI')
        self.qt = QtWidgets.QWidget()

        ui_file_path = os.path.join(
            get_package_share_directory('science'),
            'routine_gui',
            'routine_gui.ui'
            )

        uic.loadUi(ui_file_path, self.qt)
        self.task_launcher_init()
        self.qt.show() # Show the GUI


    def task_launcher_init(self):
        self.signals = Signals()

        # self.qt.packet_browser.itemSelectionChanged.connect(self.select_response_packet)
        # self.qt.pushButton_clear_packets.clicked.connect(self.clear_packets)  # Clear the packet browser
    
def main(args=None):
    rclpy.init(args=args)
    
    # Create the Qt application
    app = QtWidgets.QApplication(sys.argv)
    gui = science_routine_GUI()  # Initialize your GUI

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