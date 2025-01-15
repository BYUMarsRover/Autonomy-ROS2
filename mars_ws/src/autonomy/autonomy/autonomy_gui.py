import rclpy
from rclpy.node import Node

from PyQt5 import uic
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *

import os

from std_srvs.srv import SetBool
from rover_msgs.srv import SetFloat32, AutonomyAbort, AutonomyWaypoint
from rover_msgs.msg import AutonomyTaskInfo, RoverStateSingleton, RoverState, NavStatus, FiducialData, FiducialTransformArray, ObjectDetections
from ublox_read_2.msg import PositionVelocityTime #TODO: Uncomment this and get ublox_read_2 working, delete PositionVelocityTime from rover_msgs

class AutonomyGUI(Node):
    def __init__(self):
        # Initialize ROS2 node
        super().__init__('autonomy_gui')

        #Initialize variables
        self.start = None
        self.goal = None
        self.base_date_time = 'Base Station Date:  Time:'
        self.rover_date_time = 'Rover Date:  Time:'
        self.rover_state = 'Speed: m/s\nDirection: degrees\nNavigation State: '
        self.nav_status = 'State Machine State: \n State Machine: '
        self.base_numSV = 0
        self.rover_numSV = 0

        ################# ROS Communication #################

        # Publishers

        # Subscribers
        self.create_subscription(PositionVelocityTime, '/base/PosVelTime', self.base_GPS_info_callback, 10) #GPS info from base station
        self.create_subscription(PositionVelocityTime, '/rover/PosVelTime', self.rover_GPS_info_callback, 10) #GPS info from rover
        self.create_subscription(RoverState, "/rover_status", self.rover_state_callback, 10) #Rover state (speed, direction, navigation state)
        self.create_subscription(NavStatus, '/nav_status', self.rover_nav_status_callback, 10) #Autonomy State machine status

        # Services

        # Clients
        self.enable_autonomy_client = self.create_client(SetBool, '/autonomy/enable_autonomy')
        self.send_waypoint_client = self.create_client(AutonomyWaypoint, '/AU_waypoint_service')
        self.abort_autonomy_client = self.create_client(AutonomyAbort, '/autonomy/abort_autonomy')

        ################# GUI Creation #################

        # Load the .ui
        # file
        uic.loadUi(
            os.path.expanduser('~') + '/mars_ws/src/autonomy/autonomy_gui.ui', self)
        self.show()  # Show the GUI

    #Callbacks for Subscribers
    def base_GPS_info_callback(self, msg):
        self.base_GPS_info = msg
        self.base_numSV = msg.numSV
        base_year = msg.year
        base_month = msg.month
        base_day = msg.day
        base_hour = msg.hour
        base_min = msg.min
        base_sec = msg.sec
        self.base_date_time = f'Base Station Date: {base_month}/{base_day}/{base_year}/  Time: {base_hour}:{base_min}:{base_sec}'
        return
    
    def rover_GPS_info_callback(self, msg):
        self.rover_GPS_info = msg
        self.rover_numSV = msg.numSV
        rover_year = msg.year
        rover_month = msg.month
        rover_day = msg.day
        rover_hour = msg.hour
        rover_min = msg.min
        rover_sec = msg.sec
        self.rover_date_time = f'Rover Date: {rover_month}/{rover_day}/{rover_year}/  Time: {rover_hour}:{rover_min}:{rover_sec}'
        return

    def rover_state_callback(self, msg):
        self.rover_state_msg = msg
        self.speed = msg.speed
        self.direction = msg.direction
        navigation_state = msg.navigation_state
        if navigation_state == 0:
            self.navigation_state = 'AUTONOMOUS_STATE'
        elif navigation_state == 1:
            self.navigation_state = 'TELEOPERATION_STATE'
        elif navigation_state == 2:
            self.navigation_state = 'ARRIVAL_STATE'
        else:
            self.navigation_state = 'UNKNOWN_STATE'
        
        self.rover_state = f'Speed: {self.speed} m/s\nDirection: {self.direction} degrees\nNavigation State: {self.navigation_state}'

        return
    
    def rover_nav_status_callback(self, msg):
        self.rover_nav_status = msg
        self.state_machine_state = msg.state
        autonomous_enable = msg.auto_enable
        if autonomous_enable:
            self.autonomous_enable = 'Enabled'
        else:
            self.autonomous_enable = 'Disabled'
        self.nav_status = f'State Machine State: {self.state_machine_state}\n State Machine: {self.autonomous_enable}'

        return

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
        req = AutonomyAbort.Request()
        req.abort_status = True

        try:
            lat = float(self.latitude_input.text())
            lon = float(self.longitude_input.text())
        except ValueError:
            self.error_label.setText('Invalid latitude or longitude')
            return

        # Create a task and append to the task list
        task = AutonomyTaskInfo()
        req.lat = lat
        req.lon = lon

        #send the Request
        future = self.abort_autonomy_client.call_async(req)
        self.error_label.setText('Attempting Abort')

        #wait for response
        rclpy.spin_until_future_complete(self, future)
        if future.done() and future.result():
            response = future.result()
            if response.success:
                self.error_label.setText('Aborting task. Returning to given coordinates')
            else:
                self.error_label.setText(f'Failed to Abort: {response.message}')
        else:
            self.error_label.setText('Service call failed or did not complete')
   
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