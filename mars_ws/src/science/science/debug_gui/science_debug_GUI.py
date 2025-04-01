#!/usr/bin/python3

from PyQt5 import QtWidgets, uic, QtGui
from python_qt_binding.QtCore import QObject, Signal
import rclpy
from rover_msgs.msg import ScienceSerialTxPacket
from rclpy.node import Node
from science.function_mapping.function_map import ScienceModuleFunctionList as SMFL
from science.debug_gui.debug_widgits import *

class science_debug_GUI(Node):
    def __init__(self):
        super().__init__('science_debug_GUI')

        # Create Publisher
        self.science_serial_packet_pub = self.create_publisher(ScienceSerialTxPacket, '/science_serial_tx_request', 10)

        # Build Debug Window
        self.qt = DebugWindowWidget(self)  # Create an instance of the DebugWindow class

        self.qt.show() # Show the GUI

        self.base_ip = self.get_base_ip()

    def update_last_published_packet(self, msg: ScienceSerialTxPacket):
        self.qt.packet_display(msg)

    def packet_publish(self, msg: ScienceSerialTxPacket):
        self.science_serial_packet_pub.publish(msg)
           
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