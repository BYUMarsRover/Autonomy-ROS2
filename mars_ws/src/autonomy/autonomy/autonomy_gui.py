"""
Autonomy GUI

By: Daniel Webb and Gabe Slade - 2025

Notes: 
The QWidget Runs all the time and the ros node is spun at

"""

import rclpy
from rclpy.node import Node

from PyQt5 import uic
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from subprocess import Popen, PIPE
import sys
import os

# For Mapviz Usage
import yaml
import utm

from std_srvs.srv import SetBool
from std_msgs.msg import Header
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from rover_msgs.srv import SetFloat32, AutonomyAbort, AutonomyWaypoint
from rover_msgs.msg import AutonomyTaskInfo, PositionVelocityTime, RoverStateSingleton, RoverState, NavStatus, FiducialData, FiducialTransformArray, ObjectDetections
from rover_msgs.msg import AutonomyTaskInfo, RoverStateSingleton, RoverState, NavStatus, FiducialData, FiducialTransformArray, ObjectDetections
#from ublox_read_2.msg import PositionVelocityTime #TODO: Uncomment this and get ublox_read_2 working, delete PositionVelocityTime from rover_msgs

from ament_index_python.packages import get_package_share_directory

class AutonomyGUI(Node, QWidget):
    def __init__(self):
        # Initialize ROS2 node
        Node.__init__(self, 'autonomy_gui')

        # Initialize QWidget
        QWidget.__init__(self)
        # Load the .ui file
        uic.loadUi(os.path.expanduser('~') + '/mars_ws/src/autonomy/autonomy_gui.ui', self)
        self.show()  # Show the GUI

        # Timer to periodically spin the ROS node
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.spin_ros)
        self.timer.start(5)  # 5 millisecond interval for spinning ROS


        # Gui Buttons
        self.GNSSRadioButton.toggled.connect(self.update_leg_subselection)
        self.ArUcoRadioButton.toggled.connect(self.update_leg_subselection)
        self.ObjectRadioButton.toggled.connect(self.update_leg_subselection)

        self.Tag1RadioButton.toggled.connect(self.update_tag_selection)
        self.Tag2RadioButton.toggled.connect(self.update_tag_selection)
        self.Tag3RadioButton.toggled.connect(self.update_tag_selection)
        self.WaterBottleRadioButton.toggled.connect(self.update_tag_selection)
        self.HammerRadioButton.toggled.connect(self.update_tag_selection)

        self.EnableAutonomyButton.clicked.connect(self.enable_autonomy)
        self.DisableAutonomyButton.clicked.connect(self.disable_autonomy)
        self.AbortButton.clicked.connect(self.abort_autonomy)
        self.SendWaypointButton.clicked.connect(self.send_waypoint)

        self.PreviewMapvizButton.clicked.connect(self.preview_waypoint)
        self.ClearMapvizButton.clicked.connect(self.clear_mapviz)

        # GUI Input Fields
        self.latitude_input = self.LatitudeInput
        self.longitude_input = self.LongitudeInput

        # Set initial leg subselection options page to GNSS Page (0)
        self.legsubselectionStackedWidget.setCurrentIndex(0)

        #Initialize variables
        self.start = None
        self.goal = None
        self.base_date_time = 'Base Station Date:  Time:'
        self.rover_date_time = 'Rover Date:  Time:'
        self.rover_state = 'Speed: m/s\nDirection: degrees\nNavigation State: '
        self.nav_status = 'State Machine State: \n State Machine: '
        self.base_numSV = 0
        self.rover_numSV = 0
        self.state_machine_state = None
        self.tag_id = None

        ################# ROS Communication #################

        # Publishers
        self.path_publisher = self.create_publisher(Path, '/mapviz/path', 10)

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

        ################# Mapviz Communication Setup #################

        # Retrieve Mapviz Location
        self.declare_parameter('location', '')
        location = self.get_parameter('location').value

        # Use Location to get the lat and lon corresponding to the mapviz (0, 0) coordinate
        mapviz_params_path = os.path.join(get_package_share_directory('mapviz_tf'), 'params', 'mapviz_params.yaml')
        lat, lon = get_coordinates(mapviz_params_path, location)

        # Convert lat/lon to UTM coordinates
        utm_coords = utm.from_latlon(lat, lon)
        self.utm_easting_zero = utm_coords[0]
        self.utm_northing_zero = utm_coords[1]
        self.utm_zone_number = utm_coords[2]
        self.utm_zone_letter = utm_coords[3]

        self.current_previewed_waypoints = Path()

    def spin_ros(self):
        rclpy.spin_once(self)

    # Callbacks for Subscribers
    def base_GPS_info_callback(self, msg):
        self.base_GPS_info = msg
        self.base_numSV = msg.num_sv
        base_year = msg.year
        base_month = msg.month
        base_day = msg.day
        base_hour = msg.hour
        base_min = msg.min
        base_sec = msg.sec
        # Update Gui Fields
        self.BaseSats.setText(f'Satellites: {self.base_numSV}')
        self.BaseDate.setText(f'Date: {base_month}/{base_day}/{base_year}')
        self.BaseTime.setText(f'Time: {base_hour}:{base_min}:{base_sec}')
        self.BaseLat.setText(f'Latitude: {msg.latitude}')
        self.BaseLon.setText(f'Longitude: {msg.longitude}')
        return

    def rover_GPS_info_callback(self, msg):
        self.rover_GPS_info = msg
        self.rover_numSV = msg.num_sv
        rover_year = msg.year
        rover_month = msg.month
        rover_day = msg.day
        rover_hour = msg.hour
        rover_min = msg.min
        rover_sec = msg.sec
        # Update Gui Fields
        self.RoverSats.setText(f'Satellites: {self.rover_numSV}')
        self.RoverDate.setText(f'Date: {rover_month}/{rover_day}/{rover_year}')
        self.RoverTime.setText(f'Time: {rover_hour}:{rover_min}:{rover_sec}')
        self.RoverLat.setText(f'Latitude: {msg.latitude}')
        self.RoverLon.setText(f'Longitude: {msg.longitude}')
        return

    def rover_state_callback(self, msg): #rover status (speed, direction, navigation state)
        self.rover_state_msg = msg
        self.speed = msg.speed
        self.direction = msg.direction
        navigation_state = msg.navigation_state
        if navigation_state == 0:
            self.navigation_state = 'AUTONOMOUS'
        elif navigation_state == 1:
            self.navigation_state = 'TELEOPERATION'
        elif navigation_state == 2:
            self.navigation_state = 'ARRIVAL'
        else:
            self.navigation_state = 'UNKNOWN'
        #update gui fields
        #self.RoverState.setText(self.navigation_state)
        #self.RoverSpeed.setText(self.speed)
        #self.RoverDirection.setText(self.direction)
        return

    def rover_nav_status_callback(self, msg): #State machine status (state, auto_enable)
        self.rover_nav_status = msg
        if self.state_machine_state == None:
            self.prev_state_machine_state = self.state_machine_state
            self.PreviousStateDisplay.setText(self.prev_state_machine_state)
        self.state_machine_state = msg.state
        autonomous_enable = msg.auto_enable
        if autonomous_enable:
            self.autonomous_enable = 'Enabled'
        else:
            self.autonomous_enable = 'Disabled'

        self.CurrentStateDisplay.setText(self.state_machine_state)
        
        return

    # Callback functions for buttons
    def enable_autonomy(self):
        req = SetBool.Request()
        req.data = True
        future = self.enable_autonomy_client.call_async(req)
        self.error_label.setText('Enabling Autonomy...')
        rclpy.spin_until_future_complete(self, future)
        
        if future.result().success:
            self.error_label.setText('Autonomy Enabled')
        else:
            self.error_label.setText('Failed to Enable Autonomy')
        return

    def disable_autonomy(self):
        req = SetBool.Request()
        req.data = False
        future = self.enable_autonomy_client.call_async(req)
        self.error_label.setText('Disabling Autonomy...')
        rclpy.spin_until_future_complete(self, future)
        
        if future.result().success:
            self.error_label.setText('Autonomy Disabled')
        else:
            self.error_label.setText('Failed to Disable Autonomy')

    def preview_waypoint(self):
        # Find the x and y to be sent to mapviz
        lat = float(self.latitude_input.text())
        lon = float(self.longitude_input.text())
        utm_coords = utm.from_latlon(lat, lon)
        x = utm_coords[0] - self.utm_easting_zero
        y = utm_coords[1] - self.utm_northing_zero

        current_time = self.get_clock().now().to_msg()
        self.current_previewed_waypoints.header = Header()
        self.current_previewed_waypoints.header.stamp = current_time
        self.current_previewed_waypoints.header.frame_id = "map"

        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = current_time
        pose_stamped.header.frame_id = "map"

        pose_stamped.pose.position.x = x
        pose_stamped.pose.position.y = y
        pose_stamped.pose.position.z = 0.0

        pose_stamped.pose.orientation.x = 0.0
        pose_stamped.pose.orientation.y = 0.0
        pose_stamped.pose.orientation.z = 0.0
        pose_stamped.pose.orientation.w = 1.0
        
        self.current_previewed_waypoints.poses.append(pose_stamped)

        self.path_publisher.publish(self.current_previewed_waypoints)
        self.error_label.setText('Waypoint Sent for Preview')

    def clear_mapviz(self):

        # Clear the current previewed waypoints
        while(len(self.current_previewed_waypoints.poses) > 0):
            self.current_previewed_waypoints.poses.pop()

        msg = Path()

        current_time = self.get_clock().now().to_msg()
        msg.header = Header()
        msg.header.stamp = current_time
        msg.header.frame_id = "map"

        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = current_time
        pose_stamped.header.frame_id = "map"

        pose_stamped.pose.position.x = 0.0
        pose_stamped.pose.position.y = 0.0
        pose_stamped.pose.position.z = 0.0

        pose_stamped.pose.orientation.x = 0.0
        pose_stamped.pose.orientation.y = 0.0
        pose_stamped.pose.orientation.z = 0.0
        pose_stamped.pose.orientation.w = 1.0
        
        msg.poses = [pose_stamped]

        self.path_publisher.publish(msg)
        self.error_label.setText('Mapviz Cleared')
        
    def send_waypoint(self):
        #logic for sending waypoint
        req = AutonomyWaypoint.Request()
        req.task_list = []

        try:
            self.get_logger().info(f'Latitude: {self.latitude_input.text()}')
            self.get_logger().info(f'Longitude: {self.longitude_input.text()}')
            self.get_logger().info('in init AutonomyStateMachine')
            lat = float(self.latitude_input.text())
            lon = float(self.longitude_input.text())
        except ValueError:
            self.error_label.setText('Invalid latitude or longitude')
            return

        # Create a task and append to the task list
        task = AutonomyTaskInfo()
        task.latitude = lat
        task.longitude = lon
        if self.tag_id == None:
            self.error_label.setText('No tag selected')
            return
        else:
            task.tag_id = self.tag_id

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
        return

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
        return

    # Gui Functions
    def update_leg_subselection(self):
        if self.GNSSRadioButton.isChecked():
            self.leg_type = 'GNSS'
            self.tag_id = 'GPS_only'
            self.legsubselectionStackedWidget.setCurrentIndex(0)
        elif self.ArUcoRadioButton.isChecked():
            self.leg_type = 'ArUco'
            self.tag_id = None
            self.legsubselectionStackedWidget.setCurrentIndex(1)
        elif self.ObjectRadioButton.isChecked():
            self.leg_type = 'Object'
            self.tag_id = None
            self.legsubselectionStackedWidget.setCurrentIndex(2)
        return

    def update_tag_selection(self):
        if self.Tag1RadioButton.isChecked():
            self.tag_id = '1'
        elif self.Tag2RadioButton.isChecked():
            self.tag_id = '2'
        elif self.Tag3RadioButton.isChecked():
            self.tag_id = '3'
        elif self.WaterBottleRadioButton.isChecked():
            self.tag_id = 'bottle'
        elif self.HammerRadioButton.isChecked():
            self.tag_id = 'mallet'
        else:
            self.tag_id = None
        return

def get_coordinates(file_path, location):
    # Read the YAML file
    with open(file_path, 'r') as file:
        data = yaml.safe_load(file)
    
    # Navigate to the locations data
    locations = data['/**']['ros__parameters']['locations']
    
    # Check if the location exists
    if location in locations:
        lat = locations[location]['latitude']
        lon = locations[location]['longitude']
        return lat, lon
    else:
        return None
        
def gui_ros_spin_thread(node):
    rclpy.spin(node)

def main(args=None):
    # Initialize ROS2
    rclpy.init(args=args)

    Popen("pkill gst", shell=True, preexec_fn=os.setsid, stderr=PIPE)
    # Create QApplication
    gui_QWidget = QApplication(sys.argv)

    # Create ROS Gui
    gui_ros = AutonomyGUI()

    # ros_thread = threading.Thread(target=gui_ros_spin_thread, args=(gui_ros,), daemon=True)
    # ros_thread.start() # Start gui ROS thread

    # Run the Qt event loop
    # try:
    #     sys.exit(gui_QWidget.exec_())
    # except KeyboardInterrupt:
    #     pass
    # finally:
    #     rclpy.shutdown()
    #     ros_thread.join()

    gui_QWidget.exec_()
    rclpy.shutdown()

if __name__ == '__main__':
    main()