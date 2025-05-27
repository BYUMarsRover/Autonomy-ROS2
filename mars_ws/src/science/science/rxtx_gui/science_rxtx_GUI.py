#!/usr/bin/python3

from PyQt5 import QtWidgets, uic, QtGui
from python_qt_binding.QtCore import QObject, Signal
import rclpy
from rover_msgs.msg import ScienceSerialTxPacket
from std_msgs.msg import UInt8MultiArray, String
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import matplotlib.pyplot as plt
import numpy as np

import os
import sys

class Signals(QObject):
    science_rx_signal = Signal()
    science_tx_signal = Signal()
    # sensor_save_signal = Signal(ScienceSaveSensor)
    # FAD_save_signal = Signal(ScienceSaveFAD)
    # notes_save_signal = Signal(ScienceSaveNotes)
    # fad_intensity_signal = Signal(ScienceFADIntensity)

class science_rxtx_GUI(Node):
    def __init__(self):
        
        super().__init__('science_rxtx_GUI')
        self.qt = QtWidgets.QWidget()

        ui_file_path = os.path.join(
            get_package_share_directory('science'),
            'rxtx_gui',
            'rxtx_gui.ui'
            )

        uic.loadUi(ui_file_path, self.qt)
        self.task_launcher_init()

        # Create Publisher
        self.science_serial_tx_request_pub = self.create_publisher(ScienceSerialTxPacket, '/science/serial/tx_request', 10)

        # File Submission
        self.pub_file_contents = self.create_publisher(String, '/science/send_file', 10)

        # Create Subscriptions
        self.science_serial_tx_sub = self.create_subscription(UInt8MultiArray, '/science/serial/tx_notification', self.receive_tx_notification, 10)
        self.science_serial_rx_sub = self.create_subscription(UInt8MultiArray, '/science/serial/rx_notification', self.receive_rx_notification, 10)

        # Ensure empty
        self.clear_rxtx_windows()

        self.qt.show() # Show the GUI

    def task_launcher_init(self):
        self.signals = Signals()

        # Clear Button
        self.qt.pushButton_clear.clicked.connect(self.clear_rxtx_windows)

        # Set up form validation
        self.qt.lineEdit_function_addr.setValidator(QtGui.QIntValidator(0, 31)) # 5 bits

        # Submit Form Button
        self.qt.pushButton_submit_form.clicked.connect(self.submit_packet_form)
        self.qt.pushButton_hex_manual.clicked.connect(self.submit_manual_hexdecimal_form)
        self.qt.pushButton_ascii_manual.clicked.connect(self.submit_manual_ascii_form)

        # Submit File Button
        self.qt.file_select_button.clicked.connect(self.send_file_to_module)
        
    def append_text_to_label(self, label, text):
        label.setText(label.text() + text)

    def append_serial_traffic(self, data, hex_label, ascii_label):
        hex_text = ' '.join(f'{byte:02x}' for byte in data) + ' '
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
        self.science_serial_tx_request_pub.publish(
            ScienceSerialTxPacket(packet=self.read_hex_values(self.qt.lineEdit_hex_manual))
        )

    def submit_manual_ascii_form(self):
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

    def send_file_to_module(self):
        # Open a file selection dialog to allow the user to select a file
        file_dialog = QtWidgets.QFileDialog()
        file_dialog.setWindowTitle("Select File")
        file_dialog.setFileMode(QtWidgets.QFileDialog.ExistingFile)

        if file_dialog.exec_():
            selected_file = file_dialog.selectedFiles()[0]  # Get the selected file path
            self.get_logger().info(f"Selected file: {selected_file}")

            # Show a popup window to indicate the selected file
            msg_box = QtWidgets.QMessageBox()
            msg_box.setIcon(QtWidgets.QMessageBox.Information)
            msg_box.setWindowTitle("File Selected")
            msg_box.setText(f"Selected file: {selected_file}")
            msg_box.setStandardButtons(QtWidgets.QMessageBox.Ok)
            msg_box.exec_()

            # Call the function to update the routines with the selected file
            self.pub_file_contents.publish(String(data=selected_file))
    
    
def main(args=None):
    rclpy.init(args=args)
    
    # Create the Qt application
    app = QtWidgets.QApplication(sys.argv)
    gui = science_rxtx_GUI()  # Initialize your GUI

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