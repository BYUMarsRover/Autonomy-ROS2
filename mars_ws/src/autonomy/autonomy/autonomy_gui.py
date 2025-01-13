import rclpy
from rclpy.node import Node

from PyQt5 import uic
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *

import os

from std_srvs.srv import SetBool

class AutonomyGUI(Node):
    def __init__(self):
        # Initialize ROS2 node
        super().__init__('autonomy_gui')

        self.start = None
        self.goal = None

        ################# ROS Communication #################

        # Publishers

        # Subscribers

        # Services

        # Clients
        self.enable_autonomy_client = self.create_client(SetBool, '/autonomy/enable_autonomy')

        ################# GUI Creation #################

        # Load the .ui
        # file
        uic.loadUi(
            os.path.expanduser('~') + '/mars_ws/src/autonomy/autonomy_gui.ui', self)
        self.show()  # Show the GUI

    

    # Callback functions for buttons
    def enable_autonomy(self):
        req = SetBool.Request()
        req.data = True
        future = self.enable_autonomy_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.error_label.setText('Enabling Autonomy')
        
        if future.result().success:
            self.error_label.setText('Autonomy Enabled')
        else:
            self.error_label.setText('Failed to Enable Autonomy')

    def disable_autonomy(self):
        req = SetBool.Request()
        req.data = False
        future = self.enable_autonomy_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.error_label.setText('Disabling Autonomy')
        
        if future.result().success:
            self.error_label.setText('Autonomy Disabled')
        else:
            self.error_label.setText('Failed to Disable Autonomy')
        
    def send_waypoint(self):
        try:
            lat = float(self.latitude_input.text())
            lon = float(self.longitude_input.text())
            self.error_label.setText('Start position updated')
        except ValueError:
            self.error_label.setText('Invalid coordinates')

    

def main(args=None):
    # Initialize ROS2
    rclpy.init(args=args)
    
    # Create and run GUI
    gui = AutonomyGUI()
    
    # Spin ROS2 node
    rclpy.spin(gui)
    
    # Cleanup
    gui.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()