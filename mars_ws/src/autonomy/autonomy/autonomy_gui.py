import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
                           QLineEdit, QPushButton, QLabel)
from PyQt5.QtCore import Qt
from std_srvs.srv import SetBool
from rover_msgs.srv import SetFloat32, AutonomyAbort, AutonomyWaypoint
from rover_msgs.msg import AutonomyTaskInfo

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
        self.send_waypoint_client = self.create_client(AutonomyWaypoint, '/AU_waypoint_service')
        self.abort_autonomy_client = self.creat_client(AutonomyAbort, '/autonomy/abort_autonomy')

        ################# GUI Creation #################
        
        # Create Qt application and main window
        self.app = QApplication([])
        self.window = QWidget()
        self.window.setWindowTitle('Autonomy GUI')
        
        # Create main layout
        main_layout = QHBoxLayout()

        # Add two columns to main layout
        waypoint_column = QVBoxLayout()
        dashboard_column = QVBoxLayout()
        main_layout.addLayout(waypoint_column)
        main_layout.addLayout(dashboard_column)

        
        # Create input fields (red in template)
        self.latitude_input = QLineEdit()
        self.latitude_input.setPlaceholderText('Enter Latitude')
        waypoint_column.addWidget(self.latitude_input)
        
        self.longitude_input = QLineEdit()
        self.longitude_input.setPlaceholderText('Enter Longitude')
        waypoint_column.addWidget(self.longitude_input)
        
        # Create error message label (blue in template)
        self.error_label = QLabel()
        self.error_label.setStyleSheet('color: red;')
        self.error_label.setAlignment(Qt.AlignCenter)
        dashboard_column.addWidget(self.error_label)
        
        # Create buttons
        buttons = [
            ('Enable Autonomy', self.enable_autonomy),
            ('Disable Autonomy', self.disable_autonomy),
            ('Send Waypoint', self.send_waypoint),
            ('Abort', self.abort_autonomy)
        ]
        
        for button_text, callback in buttons:
            button = QPushButton(button_text)
            button.clicked.connect(callback)
            waypoint_column.addWidget(button)
        
        # Set layout and display window
        self.window.setLayout(main_layout)
        self.window.show()
        
        # Start Qt event loop
        self.app.exec_()


    # Callback functions for buttons
    def enable_autonomy(self):
        req = SetBool.Request()
        req.data = True
        future = self.enable_autonomy_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.error_label.setText('Aborting Task')
        
        if future.result().success:
            self.error_label.setText('Task aborted. Manual mode turned on.')
        else:
            self.error_label.setText('Failed to Abort Autonomy')
        
    def send_waypoint(self):
        #logic for sending waypoint
        req = AutonomyWaypoint.Request()
        req.task_list = []

        try:
            lat = float(self.latitude_input.text())
            lon = float(self.longitude_input.text())
        except ValueError:
            self.error_label.setText('Invalid latitude or longitude')
            return

        # Create a task and append to the task list
        task = AutonomyTaskInfo()
        task.latitude = lat
        task.longitude = lon
        task.tag_id = 'GPS_only'
        req.task_list.append(task)

        #send the Request
        future = self.send_waypoint_client.call_async(req)
        self.error_label.setText('Sending Waypoint')

        #wait for response
        rclpy.spin_until_future_complete(self, future)
        if future.done() and future.result():
            response = future.result()
            if response.success:
                self.error_label.setText('Waypoint Sent')
            else:
                self.error_label.setText(f'Failed to Send Waypoint: {response.message}')
        else:
            self.error_label.setText('Service call failed or did not complete')

    def abort_autonomy(self):
        #logic for aborting autonomy task
        req = SetBool.Request()
        req.data = True
        future = self.abort_autonomy_client.call_async(req)
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