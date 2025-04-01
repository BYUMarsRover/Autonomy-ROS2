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
import random
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
        self.clear_detail_view()

        self.task_launcher_init()

        # List to hold all packsets
        self.response_packet_list = []

        # Tracking Dialog Boxes
        self.dialog_box_packets = []

        # Create Subscriptions
        self.science_serial_rx_sub = self.create_subscription(ScienceSerialRxPacket, '/science_serial_rx_packet', self.receive_response_packet, 10)

        self.qt.show() # Show the GUI

    def extract_timestamp(self, msg: ScienceSerialRxPacket):
        return datetime.datetime.fromtimestamp(msg.timestamp).time().__str__()

    def get_icon(self, msg: ScienceSerialRxPacket):
        if msg.error_code == 0:
            return self.icon_check
        elif msg.error_code == 1:
            return self.icon_warning
        else:
            return self.icon_error
        
    def check_popup(self, msg: ScienceSerialRxPacket):
        if msg.error_code == 0:
            if self.qt.popup_message.isChecked():
                self.run_popup(msg, "Message")
        elif msg.error_code == 1:
            if self.qt.popup_warning.isChecked():
                self.run_popup(msg, "Warning")
        else:
            if self.qt.popup_error.isChecked():
                self.run_popup(msg, "Error")
        
    def run_popup(self, msg: ScienceSerialRxPacket, title):

        # Stop dups
        for packet, index in self.dialog_box_packets:
            if self.packets_equivalent(msg, packet):
                return

        dlg = QtWidgets.QDialog(self.qt)  # Create a dialog
        dlg.setWindowTitle(title)
        dlg.setWindowIcon(self.get_icon(msg))  # Set the appropriate icon

        # Create the layout for the labels
        layout_labels = QtWidgets.QVBoxLayout()

        # Add a label to display the function name
        func = SMFL.get_function_by_command_word(msg.echo[1])
        name_label = QtWidgets.QLabel(func['function_name'] if func is not None else "Unrecognized Function")
        name_label.setWordWrap(True)  # Enable word wrapping
        layout_labels.addWidget(name_label)

        # Add a label to display the message in ASCII
        ascii_text = ''.join(chr(byte) if 32 <= byte <= 126 else '#' for byte in msg.message)
        message_label = QtWidgets.QLabel(ascii_text)
        message_label.setWordWrap(True)  # Enable word wrapping
        layout_labels.addWidget(message_label)

        # Create the layout to hold the icon
        layout_icon = QtWidgets.QHBoxLayout()

        # Add the icon to the message
        icon_label = QtWidgets.QLabel()
        icon_label.setPixmap(self.get_icon(msg).pixmap(32, 32))  # Get the icon for the message
        layout_icon.addWidget(icon_label)

        # Add the labels to the layout
        layout_icon.addLayout(layout_labels)

        # Create the primary layout
        layout = QtWidgets.QVBoxLayout()
        layout.addLayout(layout_icon)

        # Add an "OK" button to dismiss the dialog
        ok_button = QtWidgets.QPushButton("OK")
        ok_button.clicked.connect(dlg.accept)  # Connect to accept the dialog
        layout.addWidget(ok_button)

        dlg.setLayout(layout)

        # Track
        self.dialog_box_packets.append((msg, len(self.response_packet_list) - 1))

        # Ensure the packet is removed from the list when the dialog is finished
        dlg.finished.connect(lambda _: self.remove_dialog_packet(msg.timestamp))

        dlg.show()

    def remove_dialog_packet(self, timestamp):
        for packet, index in self.dialog_box_packets:
            if packet.timestamp == timestamp:
                self.dialog_box_packets.remove((packet, index))

    def receive_response_packet(self, msg: ScienceSerialRxPacket):
        # Get name from ascii contents of message
        ascii_text = ''.join(chr(byte) if 32 <= byte <= 126 else '#' for byte in msg.message)
        func = SMFL.get_function_by_command_word(msg.echo[1])

        if func is not None:
            name = f"{func['function_name']} - {ascii_text}"
        else:
            name = f"Unrecognized Function - {ascii_text}"

        last_index = len(self.response_packet_list) - 1

        # Is last one a match?
        if last_index >= 0:
            if self.packets_equivalent(msg, self.response_packet_list[last_index]):
                self.inc_item(last_index)
                self.check_popup(msg) # In case it has been closed
                # Dont add another entry
                return

        # Verify not a duplicate of any open dialog box
        for packet, index in self.dialog_box_packets:
            # If currently a dialog box, 
            if self.packets_equivalent(msg, packet):
                self.inc_item(index)
                # Dont add another entry
                return
        
        # Add to the list widget
        self.response_packet_list.append(msg)
        self.add_response_message_widget(
            self.get_icon(msg),
            name,
            last_index + 1
        )

        # Check for popup
        self.check_popup(msg)

        # Selected item was the last in the list, autoselect this one
        # Get the currently selected item in the packet browser
        current_item = self.qt.packet_browser.currentItem();
        if current_item is not None and current_item.data(Qt.UserRole)["index"] == last_index:
            self.qt.packet_browser.setCurrentRow(last_index + 1)
            self.select_response_packet()

    def packets_equivalent(self, msg1: ScienceSerialRxPacket, msg2: ScienceSerialRxPacket):
        return (msg1.echo[1] == msg2.echo[1]) and (msg1.error_code == msg2.error_code) and (msg1.message == msg2.message)
    
    def inc_item(self, index):
        item = self.qt.packet_browser.item(index)
        data = item.data(Qt.UserRole)
        data["count"] += 1  # Increment the count
        item.setData(Qt.UserRole, data)  # Update the item's data
        item.setText(f"{data['count']}x {data['name']}")  # Update the displayed tex
    
    def clear_serial_monitor(self):
        self.qt.label_rx_hex.clear()
        self.qt.label_rx_ascii.clear()

    def write_serial_monitor(self, data):
        hex_text = ' '.join(f'0x{byte:02x}' for byte in data) + ' '
        ascii_text = ' '.join(chr(byte) if 32 <= byte <= 126 else '.' for byte in data) + ' '
        self.qt.label_rx_hex.setText(hex_text)
        self.qt.label_rx_ascii.setText(ascii_text)

    def add_response_message_widget(self, icon, name, index):
        item = QtWidgets.QListWidgetItem(name)
        item.setIcon(icon)  # Set the icon for the item
        item.setData(Qt.UserRole, {"index": index, "count": 1, "name": name})
        self.qt.packet_browser.addItem(item)  # Add the item to the QListWidget

    def clear_detail_view(self):
        self.qt.packet_icon.clear() # Set the icon
        self.qt.packet_name.clear() # Set the title
        self.qt.packet_timestamp.clear() # Set the timestamp
        self.qt.packet_echo.clear() # Set the echo
        self.qt.packet_error_code.clear() # Set the error code
        self.qt.packet_message_len.clear() # Set the message length
        self.clear_serial_monitor()

    def populate_detail_view(self, packet):

        # Get the matching function call
        func = SMFL.get_function_by_command_word(packet.echo[1]) # 2nd Byte is the command word

         # Set the title
        if func is None:
            self.qt.packet_name.setText("Unrecognized Function")
        else:
            self.qt.packet_name.setText(func['function_name'])

        self.qt.packet_icon.setPixmap(self.get_icon(packet).pixmap(64, 64)) # Set the icon
        self.qt.packet_timestamp.setText(self.extract_timestamp(packet)) # Set the timestamp
        self.qt.packet_echo.setText(' '.join(f'{byte:02x}' for byte in packet.echo)) # Set the echo
        self.qt.packet_error_code.setText(str(packet.error_code)) # Set the error code
        self.qt.packet_message_len.setText(str(len(packet.message))) # Set the message length
        self.write_serial_monitor(packet.message)

    def select_response_packet(self):
        item = self.qt.packet_browser.currentItem()
        if not item is None:
            packet = self.response_packet_list[item.data(Qt.UserRole)["index"]] # Get packet data
            self.populate_detail_view(packet)

    def clear_packets(self):
        self.qt.packet_browser.clear()
        self.response_packet_list = []  # Clear the list of packets
        self.dialog_box_packets = []
        self.clear_detail_view()

    def task_launcher_init(self):
        self.signals = Signals()

        self.qt.packet_browser.itemSelectionChanged.connect(self.select_response_packet)
        self.qt.pushButton_clear_packets.clicked.connect(self.clear_packets)  # Clear the packet browser
    
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