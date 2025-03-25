#!/usr/bin/python3

from PyQt5 import QtWidgets, uic, QtGui
from python_qt_binding.QtCore import QObject, Signal
import rclpy
from rover_msgs.srv import CameraControl
from rover_msgs.msg import ScienceSerialPacket
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import matplotlib.pyplot as plt
import numpy as np
import struct

import os
import sys

class DebugWindowWidget(QtWidgets.QWidget):
    def __init__(self, node, func_list):
        super(DebugWindowWidget, self).__init__()

        self.node = node

        # Find the UI file for the skeleton
        skeleton_path = os.path.join(
            get_package_share_directory('science'),
            'debug_gui',
            'skeleton.ui'
        )

        # Load the skeleton into this object
        uic.loadUi(skeleton_path, self)

        # Get the layout object
        self.layout_command = self.findChild(QtWidgets.QScrollArea, "scrollArea_command").widget().layout()

        # Add command widgets to the first scroll area
        self.command_widgets = []
        for func_definition in func_list.filter({'action_type': 'command'}):
            widget = ActionWidget(node, self, func_definition)
            widget.setObjectName(f"commandWidget_{len(self.command_widgets)}")
            self.command_widgets.append(widget)
            self.layout_command.addWidget(widget)

        # Get the layout object
        self.layout_query = self.findChild(QtWidgets.QScrollArea, "scrollArea_query").widget().layout()

        # Add query widgets to the second scroll area
        self.query_widgets = []
        for func_definition in func_list.filter({'action_type': 'query'}):
            widget = ActionWidget(node, self, func_definition)
            widget.setObjectName(f"queryWidget_{len(self.query_widgets)}")
            self.query_widgets.append(widget)
            self.layout_query.addWidget(widget)

    def packet_display(self, msg: ScienceSerialPacket):
        last_published_command_label = self.findChild(QtWidgets.QLabel, "lastSentLabel")
        last_published_command_label.setText(f"Last Packet Sent: {hex(msg.command_word)} {[ hex(x) for x in msg.operands ]}")

    def packet_modify_flags(self, command_word):
        command_word |= 0b01000000 if self.get_override_bit() else 0b00000000  # Set the Override Bit
        command_word |= 0b00100000 if self.get_ack_bit() else 0b00000000  # Set the Ack Bit
        return command_word
    
    def get_override_bit(self):
        override_box = self.findChild(QtWidgets.QCheckBox, "ovrBit")
        return override_box.isChecked()
    
    def get_ack_bit(self):
        ack_box = self.findChild(QtWidgets.QCheckBox, "ackBit")
        return ack_box.isChecked()

class ActionWidget(QtWidgets.QWidget):
    def __init__(self, node, window, func_definition):
        super(ActionWidget, self, ).__init__()

        self.node = node
        self.window = window

        # Find the UI file for the skeleton
        action_widget_path = os.path.join(
            get_package_share_directory('science'),
            'debug_gui',
            'action_widget.ui'
        )

        self.func_definition = func_definition

        # Load the action widget into this object
        uic.loadUi(action_widget_path, self)

        # Get the layout object
        self.layout = self.findChild(QtWidgets.QGroupBox, "groupBox").layout()

        # Add operand widgets to the layout
        self.operand_widgets = []
        for i in [1, 2, 3]:
            # Get the next operand in this function definition
            name = func_definition[f'operand_name_{i}']
            if name == '':
                break
            type = func_definition[f'operand_type_{i}']
            cnt = func_definition[f'operand_cnt_{i}']

            # Create the widget and add it to the layout
            widget = OperandWidget(name, type, cnt)
            widget.setObjectName(f"operandWidget_{len(self.operand_widgets)}")
            self.operand_widgets.append(widget)
            self.layout.insertWidget((len(self.operand_widgets) - 1) + 1, widget)

        # Modify the Title
        groupBox = self.findChild(QtWidgets.QGroupBox, "groupBox")
        groupBox.setTitle(f"{func_definition['function_name']} - {hex(self.get_command_word())}")

        # Modify the Description
        descLabel = self.findChild(QtWidgets.QLabel, "descriptionLabel")
        descLabel.setText(f"{func_definition['docstring']}")

        # Link the button to the function
        button = self.findChild(QtWidgets.QPushButton, "pushButton")
        button.clicked.connect(self.submit)

    def get_command_word(self):
        command_word = int(self.func_definition['function_addr'])
        command_word |= 0b10000000 if self.func_definition['action_type'] == 'command' else 0b00000000  # Set the Command Bit
        return command_word

    def submit(self):
        # Get Command Word
        command_word = self.window.packet_modify_flags(self.get_command_word())

        # Convert Operands into bytes
        operand_bytes = []
        for widget in self.operand_widgets:
            b = widget.get_operand_as_bytes()
            if b is list:
                operand_bytes.extend(b)
            else:
                operand_bytes.append(b)

        self.node.packet_publish(
            ScienceSerialPacket(
                command_word=command_word,
                operands=operand_bytes
            )
        )

class OperandWidget(QtWidgets.QWidget):
    def __init__(self, name, type, cnt):
        super(OperandWidget, self).__init__()

        # Find the UI file for the skeleton
        operand_widget_path = os.path.join(
            get_package_share_directory('science'),
            'debug_gui',
            'operand.ui'
        )

        self.type = type

        # Load the action widget into this object
        uic.loadUi(operand_widget_path, self)

        # Update the title
        groupBox = self.findChild(QtWidgets.QGroupBox, "groupBox")
        groupBox.setTitle(name)

        # Set Data Validation
        self.lineEdit = self.findChild(QtWidgets.QLineEdit, "lineEdit")
        if type == 'uint8_t':
            self.lineEdit.setValidator(QtGui.QIntValidator(0, 2**8 - 1))
        elif type == 'uint16_t':
            self.lineEdit.setValidator(QtGui.QIntValidator(0, 2**16 - 1))
        elif type == 'uint32_t':
            self.lineEdit.setValidator(QtGui.QIntValidator(0, 2**31 - 1))
        elif type == 'int8_t':
            self.lineEdit.setValidator(QtGui.QIntValidator(-(2*7), (2*7)-1))
        elif type == 'float':
            self.lineEdit.setValidator(QtGui.QDoubleValidator())

    def get_operand_as_bytes(self):
        if self.type == 'uint8_t' \
        or self.type == 'uint16_t' \
        or self.type == 'uint32_t' \
        or self.type == 'int8_t':
            return int(self.lineEdit.text())
        elif self.type == 'float':
            return struct.pack('f', float(self.lineEdit.text()))
        else:
            raise Exception(f"Unknown type in OperandWidget: {self.type}")
        
class Signals(QObject):
    serial_signal = Signal(ScienceSerialPacket)

class science_debug_GUI(Node):
    def __init__(self):
        super().__init__('science_debug_GUI')

        # Create a subscriber to the science_serial_packet topic
        self.signals = Signals()
        self.signals.serial_signal.connect(self.update_last_published_packet)
        self.science_serial_packet_sub = self.create_subscription(ScienceSerialPacket, '/science_serial_packet', self.signals.serial_signal.emit, 10)

        # Create Publisher
        self.science_serial_packet_pub = self.create_publisher(ScienceSerialPacket, '/science_serial_packet', 10)

        # Build Debug Window
        self.qt = DebugWindowWidget(self, ScienceModuleFunctionList())  # Create an instance of the DebugWindow class

        self.qt.show() # Show the GUI

        self.base_ip = self.get_base_ip()

    def update_last_published_packet(self, msg: ScienceSerialPacket):
        self.qt.packet_display(msg)

    def packet_publish(self, msg: ScienceSerialPacket):
        self.science_serial_packet_pub.publish(msg)
           
    def get_base_ip(self):
        ip = os.getenv("BASE_ADDRESS")
        if ip is None:
            ip = "192.168.1.65"
        return ip
    
from csv import DictReader

class ScienceModuleFunctionList:

    def __init__(self):
        path = os.path.join(
            get_package_share_directory('science'),
            'debug_gui',
            'science_module_function_map.csv'
        )
        self.function_list = self.load_function_map(path)

    def load_function_map(self, filename):
        with open(filename, 'r') as f:
            dict_reader = DictReader(f)
            return list(dict_reader)
        
    def get_all(self):
        return self.function_list
    
    def filter(self, args):
        """Filter the function list based on the provided arguments."""
        filtered_list = []
        for func in self.function_list:
            skip = False
            for k, v in args.items():
                if not v == func[k]:
                    skip = True
                    break
            if not skip:
                filtered_list.append(func)
        return filtered_list
    
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