"""
Autonomy GUI

By: Daniel Webb and Gabe Slade - 2025

Notes: 
The QWidget runs all the time and the ros node is spun in a different thread

"""
import rclpy
from rclpy.node import Node

from PyQt5 import uic
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
import sys
import os
import numpy as np

# Used by Mapviz and others
import yaml
import utm

from std_srvs.srv import SetBool
from std_msgs.msg import Header, Int8
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Point
from rover_msgs.srv import AutonomyAbort, AutonomyWaypoint, OrderPath, SetFloats, SetFloat32, OrderAutonomyWaypoint, PlanPath
from rover_msgs.msg import AutonomyTaskInfo, RoverStateSingleton, NavState, RoverState, FiducialData, FiducialTransformArray, ObjectDetections, MobilityAutopilotCommand, MobilityVelocityCommands, MobilityDriveCommand, IWCMotors
from sensor_msgs.msg import Image
from ament_index_python.packages import get_package_share_directory

import threading


class AutonomyGUI(Node, QWidget):
    # This signal is used to isolate the updating of graphical elements from the ROS callbacks
    ros_signal = pyqtSignal(str, str)

    def __init__(self):
        # Initialize ROS2 node
        Node.__init__(self, 'autonomy_gui')

        # Initialize QWidget
        QWidget.__init__(self)
        # Load the .ui file
        uic.loadUi(os.path.expanduser('~') + '/mars_ws/src/autonomy/autonomy_gui.ui', self)
        self.show()  # Show the GUI
        
        ############################## GUI #############################
        self.ros_signal.connect(self.gui_setText)
        #################### GUI Button Connections ####################
        # Connect Leg Type Radio Buttons to update_leg_subselection function
        self.GNSSRadioButton.toggled.connect(self.update_leg_subselection)
        self.ArUcoRadioButton.toggled.connect(self.update_leg_subselection)
        self.ObjectRadioButton.toggled.connect(self.update_leg_subselection)
        for i in range(1, 4):
            getattr(self, f'Tag{i}RadioButton').toggled.connect(self.update_tag_selection)
        self.BottleRadioButton.toggled.connect(self.update_tag_selection)
        self.MalletRadioButton.toggled.connect(self.update_tag_selection)

        # Autonomy Control Buttons
        self.EnableAutonomyButton.clicked.connect(self.enable_autonomy)
        self.DisableAutonomyButton.clicked.connect(self.disable_autonomy)
        self.AbortButton.clicked.connect(self.abort_autonomy)
        self.SendWaypointButton.clicked.connect(self.send_waypoint)
        self.ClearWaypointButton.clicked.connect(self.clear_waypoint)

        # Mapviz Buttons
        self.PreviewMapvizButton.clicked.connect(self.preview_waypoint)
        # self.PlanOrderMapvizButton.clicked.connect(self.plan_order_mapviz_service_call)
        self.ClearMapvizButton.clicked.connect(self.clear_mapviz)

        # Mobility Control Buttons
        self.SetGainsButton.clicked.connect(self.set_gains)
        self.SetSpeedConstantButton.clicked.connect(self.set_speed)

        # Waypoint List Functions
        self.AddWaypointButton.clicked.connect(self.add_waypoint)
        self.PlanOrderButton.clicked.connect(self.plan_order_service_call)
        self.RemoveSelectedWaypointButton.clicked.connect(self.remove_selected_waypoint)
        for i in range(1, 8):
            getattr(self, f'WP{i}RadioButton').toggled.connect(self.update_selected_waypoint)
        self.PlanPathButton.clicked.connect(self.request_plan_path)
        self.selected_waypoint = None

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
        self.obj_distance = None
        self.obj_angle = None
        self.obj_alpha_lpf = 0.5
        self.aruco_alpha_lpf = 0.5
        self.aruco_tag_distance = None
        self.course_heading_error = None
        self.state_machine_list_string = ''
        self.autopilot_cmds_msg = None
        self.waypoints = [] # Stores the waypoints in the format [waypoint_number, tag_id, latitude, longitude, status]
        self.selected_waypoint = None
        self.selected_waypoint_to_send = None
        self.selected_waypoint_for_path_planning = None


        # This should return a list like this: [lat, lon] and can be used for the plan path to selected waypoint
        # lat, lon = self.waypoints[self.selected_waypoint - 1][2:4]

        ################# ROS Communication #################

        # Publishers
        self.path_publisher = self.create_publisher(Path, '/mapviz/path', 10)

        # Subscribers
        self.create_subscription(RoverStateSingleton, '/odometry/rover_state_singleton', self.rover_state_singleton_callback, 10) #Rover GPS and Heading
        self.create_subscription(NavState, '/nav_state', self.nav_state_callback, 10) # Navigation state (speed, direction, navigation state)
        self.create_subscription(RoverState, '/rover_state', self.rover_state_callback, 10) # Autonomy State machine state
        self.create_subscription(FiducialTransformArray, '/aruco_detect_logi/fiducial_transforms', self.ar_tag_callback, 10) #Aruco Detection
        self.create_subscription(ObjectDetections, '/zed/object_detection', self.obj_detect_callback, 10) #Object Detection
        self.create_subscription(MobilityAutopilotCommand, '/mobility/autopilot_cmds', self.autopilot_cmds_callback, 10) #What mobility/path_manager is publishing
        self.create_subscription(MobilityVelocityCommands, '/mobility/rover_vel_cmds', self.vel_cmds_callback, 10) #What mobility/autopilot_manager is publishing
        self.create_subscription(MobilityDriveCommand, '/mobility/wheel_vel_cmds', self.wheel_vel_cmds_callback, 10) #What mobility/wheel_manager is publishing
        self.create_subscription(IWCMotors, '/mobility/auto_drive_cmds', self.auto_drive_cmds_callback, 1) 
        self.create_subscription(PlanPath.Response, '/path_plan_response', self.plan_path_response_callback, 10) # Allows the path planner node to notify when the path is ready
        self.create_subscription(Image, '/image_raw', self.image_callback, 10) # Image from the camera

        # Services

        # Clients
        # Enables Autonomy
        self.enable_autonomy_client = self.create_client(SetBool, '/autonomy/enable_autonomy')
        # Sends a single waypoint in the case that path planning is not being used
        self.send_waypoint_client = self.create_client(AutonomyWaypoint, '/AU_waypoint_service')
        # Commands the path_planner node to send the waypoints corresponding to the planned path to the state machine
        self.send_path_client = self.create_client(SetBool, '/send_path_service')
        # Clears the state machine's waypoint if there is one
        self.clear_waypoint_client = self.create_client(SetBool, '/AU_clear_waypoint_service')
        # Reorders the list of waypoints that has been added to the gui using the path planner node
        self.plan_order_client = self.create_client(OrderAutonomyWaypoint, '/plan_order')
        # Aborts Autonomy mission
        self.abort_autonomy_client = self.create_client(AutonomyAbort, '/autonomy/abort_autonomy')
        # Requests that the path planner plans the path to the selected waypoint
        self.plan_path_client = self.create_client(PlanPath, '/plan_path')

        #NOTE: depricated until mapviz capability added back
        # self.plan_order_mapviz_client = self.create_client(OrderPath, '/plan_order_mapviz')

        # Clients used for tunning controller gains TODO: remove once tuned for competition
        self.set_autopilot_gains_client = self.create_client(SetFloats, '/mobility/autopilot_manager/set_gains')
        self.set_aruco_autopilot_gains_client = self.create_client(SetFloats, '/mobility/aruco_autopilot_manager/set_gains')
        self.set_speed_client = self.create_client(SetFloat32, '/mobility/drive_manager/set_speed')

        # Timer to run check if we have recieved information from various sources recently
        # for the purpose of clearing the information if it is not recent
        self.timepoints_timer = self.create_timer(0.5, self.check_timepoints)
        self.rover_state_singleton_timepoint = None

        ################# Mapviz Communication Setup #################

        # Retrieve Mapviz Location
        self.declare_parameter('MAPVIZ_LOCATION', 'hanksville')
        mapviz_location = self.get_parameter('MAPVIZ_LOCATION').value

        # Use Location to get the lat and lon corresponding to the mapviz (0, 0) coordinate
        mapviz_origins_path = os.path.join(get_package_share_directory('mapviz_tf'), 'params', 'mapviz_origins.yaml')
        # Open mapviz origins file
        with open(mapviz_origins_path, 'r') as file:
            mapviz_origins = yaml.safe_load(file)
        for location in mapviz_origins: # iterate over dictionaries to find the lat/lon of the location
            if location['name'] == mapviz_location:
                self.origin_latlon = np.array([location['latitude'], location['longitude']])
                break
        # Get mapviz origin in UTM coordinates
        self.origin = np.array((utm.from_latlon(*self.origin_latlon)[0:2])[::-1])

        self.utm_easting_zero = self.origin[1]
        self.utm_northing_zero = self.origin[0]

        # Initialize the current previewed waypoints
        # Stored in lat/lon format
        self.current_previewed_waypoints = Path() #NOTE: used by mapviz

    # Clears displays in the gui if information stops being received.
    def check_timepoints(self):
        if self.rover_state_singleton_timepoint is not None:
            if self.get_clock().now().to_msg().sec - self.rover_state_singleton_timepoint > 1:
                self.clear_rover_state_singleton_info()
    
    def clear_rover_state_singleton_info(self):
        self.ros_signal.emit('RoverStateMapYaw', 'Map Yaw: ...')
        self.ros_signal.emit('RoverStateLat','Latitude: ...')
        self.ros_signal.emit('RoverStateLon', 'Longitude: ...')
        return

    ################# Callbacks for Subscribers #################

    def nav_state_callback(self, msg): #rover status (speed, direction, navigation state)
        self.rover_state_msg = msg
        self.speed = msg.speed
        self.direction = msg.direction
        nav_state = msg.navigation_state
        if nav_state == 0:
            self.nav_state = 'AUTONOMOUS'
        elif nav_state == 1:
            self.nav_state = 'TELEOPERATION'
        elif nav_state == 2:
            self.nav_state = 'ARRIVAL'
            if self.selected_waypoint_to_send is not None:
                self.waypoints[self.selected_waypoint_to_send -1][4] = 'COMPLETE'
        else:
            self.nav_state = 'UNKNOWN'
        # Update GUI fields
        self.ros_signal.emit('PreviousNavStateDisplay', self.CurrentNavStateDisplay.text())
        self.ros_signal.emit('CurrentNavStateDisplay', self.nav_state)
        return

    def rover_state_callback(self, msg): #State machine status (state, auto_enable)
        #Update previous state and state list
        if self.state_machine_state != None and msg.state != self.state_machine_state:
            self.state_machine_list_string = f'{msg.state}\n' + self.state_machine_list_string
            self.ros_signal.emit('PreviousStatesList', self.state_machine_list_string) 

            self.prev_state_machine_state = self.state_machine_state

        #Show the first state on the previous state column
        if self.state_machine_state == None:
            self.state_machine_list_string = f'{msg.state}\n'
            self.ros_signal.emit('PreviousStatesList', self.state_machine_list_string)

        #Update current state and autonomous enable
        self.state_machine_state = msg.state
        autonomous_enable = msg.auto_enable
        if autonomous_enable:
            self.autonomous_enable = 'Enabled'
        else:
            self.autonomous_enable = 'Disabled'
        
        return

    def ar_tag_callback(self, msg):

        # Clear the string
        aruco_text = ''

        if len(msg.transforms) >= 1: #TODO: if we happpen to see 2, this will not run... JM - Unsure what this was used for. 
            aruco_x = msg.transforms[0].transform.translation.x
            aruco_z = msg.transforms[0].transform.translation.z
            aruco_dist = np.sqrt(aruco_x ** 2 + aruco_z ** 2)
            aruco_angle = np.arctan(aruco_x / aruco_z)
            if self.aruco_tag_distance is None:
                self.aruco_tag_distance = aruco_dist
                self.aruco_tag_angle = aruco_angle
            else:
                #Low pass filter the distance and heading information
                self.aruco_tag_distance = self.aruco_tag_distance * self.aruco_alpha_lpf + aruco_dist * (1 - self.aruco_alpha_lpf)
                self.aruco_tag_angle = self.aruco_tag_angle * self.aruco_alpha_lpf + aruco_angle * (1 - self.aruco_alpha_lpf)
            
            #TODO: When we have heading data, add it to the angle here to ge the correct angle
            aruco_text = "Tag is {}m away at {} deg.".format(round(self.aruco_tag_distance, 2), round(np.rad2deg(self.aruco_tag_angle), 2))

            # Check if the tag id is correct, compare by converting to string
            if str(msg.transforms[0].fiducial_id) == self.tag_id:
                aruco_text = aruco_text + f" Correct tagID: {self.tag_id}"
            else:
                aruco_text = aruco_text + f"Incorrect tagID: {msg.transforms[0].fiducial_id}, Correct id: {self.tag_id}"
            
            self.ros_signal.emit('ArucoStatus', aruco_text)
        return
    
    def obj_detect_callback(self, msg):
        obj_name = None
        objects_string = ''
        for obj in msg.objects:
            if obj.label == 1:
                obj_name = 'Bottle'
            elif obj.label == 2:
                obj_name = 'Mallet'
            else:
                obj_name = 'Unknown'

            # Low-pass filter the distance and heading information
            obj_dist = np.sqrt((obj.y) ** 2 + (obj.x) ** 2)
            obj_ang = -np.arctan(obj.y / obj.x)
            if self.obj_distance is None:
                self.obj_distance = obj_dist
                self.obj_angle = obj_ang
            else:
                self.obj_distance = self.obj_distance * self.obj_alpha_lpf + obj_dist * (1 - self.obj_alpha_lpf)
                self.obj_angle = self.obj_angle * self.obj_alpha_lpf + obj_ang * (1 - self.obj_alpha_lpf)

            #round to 2 places and convert angle to degrees
            obj_distance = round(self.obj_distance, 2)
            obj_angle = round(np.rad2deg(self.obj_angle), 2)

            objects_string = objects_string + f'{obj_name}: conf: {obj.confidence}, dist: {obj_distance} m @ {obj_angle} deg \n'

        self.ros_signal.emit('ObjStatus', objects_string)
        return
    
    def image_callback(self, msg):
        data = msg.data
        # Convert the image data to a QImage and display it in the QLabel
        image = QImage(data, msg.width, msg.height, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(image)
        self.CameraFeed.setPixmap(pixmap)
        self.CameraFeed.setScaledContents(True)
        return
    
    ################# Callbacks for Mobility #################
    def autopilot_cmds_callback(self, msg):
        self.autopilot_cmds_msg = msg
        self.setAutopilotString(self.autopilot_cmds_msg)        
        return

    #helper function for autopilot_cmds_callback and vel_cmds_callback
    def setAutopilotString(self, msg):
        if self.course_heading_error is None:
            autopilot_cmds_string = f'E_lin: {msg.distance_to_target:.1f}m, cw N: {np.rad2deg(msg.course_angle):.1f}°'
        else:
            autopilot_cmds_string = f'E_lin: {msg.distance_to_target:.1f}m, cw N: {np.rad2deg(msg.course_angle):.1f}°, E_ang: {np.rad2deg(self.course_heading_error):.1f}°'
        self.ros_signal.emit('AutopilotCmds', autopilot_cmds_string)
        return
    
    def vel_cmds_callback(self, msg):
        self.course_heading_error = msg.course_heading_error
        vel_cmds_string = f'Lin Vel: {round(msg.u_cmd, 2)}, Ang Vel: {round(msg.omega_cmd, 2)}'
        self.ros_signal.emit('VelocityCmds', vel_cmds_string)

        # If we have a course heading error, update the autopilot string
        if self.autopilot_cmds_msg is not None:
            self.setAutopilotString(self.autopilot_cmds_msg)

        return

    def wheel_vel_cmds_callback(self, msg):
        wheel_vel_cmds_string = f'LW Speed: {round(msg.lw, 2)}, RW Speed: {round(msg.rw, 2)}'
        self.ros_signal.emit('WheelVelocityCmds', wheel_vel_cmds_string)
        return

    def auto_drive_cmds_callback(self, msg):
        #Format the IWC wheel commands into a string
        IWC_cmd_string = ''
        #For each wheel, put a negative in the string if direction is False
        if msg.left_front_dir:
            IWC_cmd_string = IWC_cmd_string + f'LFW: {round(msg.left_front_speed, 2)}'
        else:
            IWC_cmd_string = IWC_cmd_string + f'LFW: -{round(msg.left_front_speed, 2)}'
        if msg.left_middle_dir:
            IWC_cmd_string = IWC_cmd_string + f', LMW: {round(msg.left_middle_speed, 2)}'
        else:
            IWC_cmd_string = IWC_cmd_string + f', LMW: -{round(msg.left_middle_speed, 2)}'
        if msg.left_rear_dir:
            IWC_cmd_string = IWC_cmd_string + f', LRW: {round(msg.left_rear_speed, 2)}'
        else:
            IWC_cmd_string = IWC_cmd_string + f', LRW: -{round(msg.left_rear_speed, 2)}'
        if msg.right_front_dir:
            IWC_cmd_string = IWC_cmd_string + f', RFW: {round(msg.right_front_speed, 2)}'
        else:
            IWC_cmd_string = IWC_cmd_string + f', RFW: -{round(msg.right_front_speed, 2)}'
        if msg.right_middle_dir:
            IWC_cmd_string = IWC_cmd_string + f', RMW: {round(msg.right_middle_speed, 2)}'
        else:
            IWC_cmd_string = IWC_cmd_string + f', RMW: -{round(msg.right_middle_speed, 2)}'
        if msg.right_rear_dir:
            IWC_cmd_string = IWC_cmd_string + f', RRW: {round(msg.right_rear_speed, 2)}'
        else:
            IWC_cmd_string = IWC_cmd_string + f', RRW: -{round(msg.right_rear_speed, 2)}'

        self.ros_signal.emit('IWCCmds', IWC_cmd_string)
        
        return

    # Callback functions for buttons
    def enable_autonomy(self):
        req = SetBool.Request()
        req.data = True
        if self.selected_waypoint_to_send is not None:
            self.waypoints[self.selected_waypoint_to_send -1][4] = 'ACTIVE'
        self.update_waypoint_list()
        future = self.enable_autonomy_client.call_async(req)
        self.ros_signal.emit('logger_label', 'Enabling Autonomy...')

    def disable_autonomy(self):
        req = SetBool.Request()
        req.data = False
        future = self.enable_autonomy_client.call_async(req)
        self.ros_signal.emit('logger_label', 'Disabling Autonomy...')

    # This sends the waypoint to mapviz for preview
    def preview_waypoint(self):
        # Find the x and y to be sent to mapviz
        lat = float(self.latitude_input.text())
        lon = float(self.longitude_input.text())

        current_time = self.get_clock().now().to_msg()
        self.current_previewed_waypoints.header = Header()
        self.current_previewed_waypoints.header.stamp = current_time
        self.current_previewed_waypoints.header.frame_id = "map"

        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = current_time
        pose_stamped.header.frame_id = "map"

        pose_stamped.pose.position.x = lat
        pose_stamped.pose.position.y = lon
        pose_stamped.pose.position.z = 0.0

        pose_stamped.pose.orientation.x = 0.0
        pose_stamped.pose.orientation.y = 0.0
        pose_stamped.pose.orientation.z = 0.0
        pose_stamped.pose.orientation.w = 1.0
        
        self.current_previewed_waypoints.poses.append(pose_stamped)

        self.path_publisher.publish(
            path_to_utm(self.current_previewed_waypoints, 
                        self.utm_easting_zero, 
                        self.utm_northing_zero)
            )
        
        self.ros_signal.emit('logger_label', 'Waypoint Sent for Preview')

    # This adds the waypoint to the waypoint list that is held in the autonomy gui
    def add_waypoint(self):
        try:
            self.ros_signal.emit('logger_label', 'Adding Waypoint...')
            lat = float(self.latitude_input.text())
            lon = float(self.longitude_input.text())
        except ValueError:
            self.ros_signal.emit('logger_label', 'Invalid latitude or longitude')
            return
        waypoint = [int(len(self.waypoints) + 1), self.tag_id, lat, lon, 'IDLE']

        self.waypoints.append(waypoint)
        self.update_waypoint_list()

    def remove_selected_waypoint(self):
        if self.selected_waypoint is None:
            self.ros_signal.emit('logger_label', 'No waypoint selected')
            return
        if self.selected_waypoint > len(self.waypoints):
            self.ros_signal.emit('logger_label', 'Empty Waypoint Selected')
            return
        
        if self.selected_waypoint_to_send == self.selected_waypoint:
            self.selected_waypoint_to_send = None

        # Update the waypoint numbers
        for i in range(len(self.waypoints)):
            if self.waypoints[i][0] > self.waypoints[self.selected_waypoint - 1][0]:
                self.waypoints[i][0] -= 1

        # Remove the selected waypoint
        self.waypoints.pop(self.selected_waypoint - 1)

        self.selected_waypoint = None
        self.update_waypoint_list()

        # Clear the radio buttons for update_selected_waypoint function to work
        for i in range(1, 9):
            getattr(self, f'WP{i}RadioButton').setAutoExclusive(False)
            getattr(self, f'WP{i}RadioButton').setChecked(False)
            getattr(self, f'WP{i}RadioButton').setAutoExclusive(True)

    # This finds which toggle button was selected and sets the selected_waypoint variable if it is a waypoint
    def update_selected_waypoint(self):
        for i in range(1, 9):
            if getattr(self, f'WP{i}RadioButton').isChecked() and i <= len(self.waypoints):
                self.selected_waypoint = i
                break
            else:
                self.selected_waypoint = None
    
    # Commands path planner to plan a path to the waypoint currently selected in the gui
    def request_plan_path(self):
        self.selected_waypoint_for_path_planning = self.selected_waypoint

        # Error Handling
        if not self.plan_path_client.wait_for_service(timeout_sec=2.0):
            self.ros_signal.emit('logger_label', "Plan Path service is unavailable.")
            return
        if self.selected_waypoint_for_path_planning is None:
            self.ros_signal.emit('logger_label', 'No waypoint selected')
            return
        if self.waypoints[self.selected_waypoint_for_path_planning - 1][4] == 'PATH READY':
            self.ros_signal.emit('logger_label', 'Path already planned')
            return

        tag_id, lat, lon = self.waypoints[self.selected_waypoint_for_path_planning -1][1:4] # Extract tag_id and lat/lon of the selected waypoint
        request = PlanPath.Request()
        request.goal.latitude = lat
        request.goal.longitude = lon
        request.goal.tag_id = tag_id

        future = self.plan_path_client.call_async(request)
        future.add_done_callback(self.request_plan_path_callback)

    def request_plan_path_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.ros_signal.emit('logger_label', response.message)
                self.waypoints[self.selected_waypoint_for_path_planning - 1][4] = 'PLANNING'
            else:
                self.ros_signal.emit('logger_label', response.message)
                self.waypoints[self.selected_waypoint_for_path_planning - 1][4] = 'IDLE'
        except Exception as e:
            self.get_logger().error(f'Plan Path Service call failed! {e}')
            self.ros_signal.emit('logger_label', f'Plan Path Service call failed! (See Logger)')

        self.update_waypoint_list()

    def plan_path_response_callback(self, msg):
        if msg.success:
            # Ensure that any previously planned path statuses are cleared as that path will be overriden
            for wp in self.waypoints:
                if wp[4] != 'COMPLETE':
                    wp[4] = 'IDLE'
            self.waypoints[self.selected_waypoint_for_path_planning - 1][4] = 'PATH READY'
            self.ros_signal.emit('logger_label', msg.message)
        else:
            self.waypoints[self.selected_waypoint_for_path_planning - 1][4] = 'IDLE'
            self.ros_signal.emit('logger_label', msg.message)
        self.update_waypoint_list()
        return

    # This updated the display of the waypoint list in the gui
    def update_waypoint_list(self):
        count = 0
        for waypoint in self.waypoints:
            count += 1

            # Set attributes
            getattr(self, f'WP{count}Label').setText(str(waypoint[0]))
            getattr(self, f'TagID{count}Label').setText(str(waypoint[1]))
            getattr(self, f'Lat{count}Label').setText(str(waypoint[2]))
            getattr(self, f'Lon{count}Label').setText(str(waypoint[3]))
            getattr(self, f'Status{count}Label').setText(str(waypoint[4]))

            # Formatting Status Entry
            match str(waypoint[4]):
                case 'IDLE':
                    getattr(self, f'Status{count}Label').setStyleSheet('color:rgb(150, 150, 150); \
                                                                        font-weight: bold;')
                case 'ACTIVE':
                    getattr(self, f'Status{count}Label').setStyleSheet('color:rgb(0, 61, 216);')
                case 'COMPLETE':
                    getattr(self, f'Status{count}Label').setStyleSheet('color:rgb(0, 255, 34); \
                                                                        font-weight: bold;')
                case _:
                    getattr(self, f'Status{count}Label').setStyleSheet('color:rgb(0, 0, 0);')

        for i in range(count + 1, 9):
            # Set Empty entries
            getattr(self, f'WP{i}Label').setText(' ')
            getattr(self, f'TagID{i}Label').setText(' ')
            getattr(self, f'Lat{i}Label').setText(' ')
            getattr(self, f'Lon{i}Label').setText(' ')
            getattr(self, f'Status{i}Label').setText(' ')

        return

    def plan_order_service_call(self):
        req = OrderAutonomyWaypoint.Request()
        for waypoint in self.waypoints:
            task = AutonomyTaskInfo()
            task.tag_id = waypoint[1]
            task.latitude = waypoint[2]
            task.longitude = waypoint[3]
            req.ids.append(Int8(data=waypoint[0]))
            req.task_list.append(task)

        future = self.plan_order_client.call_async(req)
        self.ros_signal.emit('logger_label', 'Planning order...')
        future.add_done_callback(self.plan_order_service_callback)
    
    def plan_order_service_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.waypoints = []
                for waypoint in response.task_list:
                    self.waypoints.append([0, waypoint.tag_id, waypoint.latitude, waypoint.longitude, 'IDLE'])
                for i in range(len(response.ids)):
                    self.waypoints[i][0] = response.ids[i].data
                self.update_waypoint_list()
                self.ros_signal.emit('logger_label', response.message)
            else:
                self.ros_signal.emit('logger_label', "Failed to plan order")
        except Exception as e:
            self.ros_signal.emit('logger_label', f'Plan Order Service call failed!')
    
    # Sends the selected waypoint to the state machine either via the path planner node or directly
    def send_waypoint(self):
        self.selected_waypoint_to_send = self.selected_waypoint
        # Error Handling
        if self.selected_waypoint_to_send is None:
            self.ros_signal.emit('logger_label', 'No waypoint selected!')
            return
        
        # If the waypoint has a path planned, send the path
        if self.waypoints[self.selected_waypoint_to_send - 1][4] == 'PATH READY':
            self.waypoints[self.selected_waypoint_to_send - 1][4] = 'SENDING'
            if not self.send_path_client.wait_for_service(timeout_sec=2.0) and not self.send_waypoint_client.wait_for_service(timeout_sec=2.0):
                self.ros_signal.emit('logger_label', 'Send path service or AU_waypoint_service unavailable!')
                return
            future = self.send_path_client.call_async(SetBool.Request(data=True))
            future.add_done_callback(self.send_waypoint_callback)

        # If the waypoint does not have a path planned, send the single waypoint directly
        else:
            if not self.send_waypoint_client.wait_for_service(timeout_sec=2.0):
                self.ros_signal.emit('logger_label', '/AU_waypoint_service service unavailable!')
                return
            self.waypoints[self.selected_waypoint_to_send - 1][4] = 'SENDING'
            tag_id, lat, lon = self.waypoints[self.selected_waypoint_to_send -1][1:4] # Extract tag_id and lat/lon of the selected waypoint
            req = AutonomyWaypoint.Request()
            req.task_list.append(AutonomyTaskInfo(tag_id=tag_id, latitude=lat, longitude=lon))
            future = self.send_waypoint_client.call_async(req)
            future.add_done_callback(self.send_waypoint_callback)

        self.update_waypoint_list()
        return

    def send_waypoint_callback(self, future):
        try:
            response = future.result()
            if response.success:
                if self.selected_waypoint_to_send is not None:
                    self.waypoints[self.selected_waypoint_to_send -1][4] = 'READY'
                self.ros_signal.emit('logger_label', response.message)
            else:
                if self.selected_waypoint_to_send is not None:
                    self.waypoints[self.selected_waypoint_to_send -1][4] = 'SEND FAILURE'
                self.ros_signal.emit('logger_label', response.message)
        except Exception as e:
            self.get_logger().error(f'Send Waypoint Service call failed! {e}')
            self.ros_signal.emit('logger_label', f'Send Waypoint Service call failed! (see logger)')
        
        self.update_waypoint_list()
        return

    # This removes the waypoint from the state machine
    def clear_waypoint(self):
        req = SetBool.Request()
        req.data = True
        future = self.clear_waypoint_client.call_async(req)
        self.ros_signal.emit('logger_label', 'Removing Last Waypoint')
        future.add_done_callback(self.clear_waypoint_callback)

    def clear_waypoint_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.ros_signal.emit('logger_label', response.message)
            else:
                self.ros_signal.emit('logger_label', "Failed to remove waypoint")
        except Exception as e:
            self.ros_signal.emit('logger_label', f'Remove Waypoint Service call failed!')

    # This calls the abort service
    def abort_autonomy(self):
        #logic for aborting autonomy task
        req = AutonomyAbort.Request()
        req.abort_status = True

        try:
            lat = float(self.latitude_input.text())
            lon = float(self.longitude_input.text())
        except ValueError:
            self.ros_signal.emit('logger_label', 'Invalid latitude or longitude')
            return

        # Create a task and append to the task list
        task = AutonomyTaskInfo()
        req.lat = lat
        req.lon = lon

        # Send the Abort Request
        future = self.abort_autonomy_client.call_async(req)
        self.ros_signal.emit('logger_label', 'Attempting Abort')

    # This is a service that adjusts the drive manager's max speed, different speeds should be used for local navigation vs GNSS navigation
    def set_speed(self):
        req = SetFloat32.Request()
        req.data = float(self.SpeedConstantInput.text())
        future = self.set_speed_client.call_async(req)
        self.ros_signal.emit('logger_label', 'Sending Speed Constant...')
        future.add_done_callback(self.set_speed_callback)

    def set_speed_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.ros_signal.emit('logger_label', response.message)
            else:
                self.ros_signal.emit('logger_label', "Failed! Must be in range (0-10)")
        except Exception as e:
            self.ros_signal.emit('logger_label', f'Send Speed Constant Service call failed!')

    def set_gains(self):

        try:
            kp_linear = float(self.kpLinearInput.text())
            kp_angular = float(self.kpAngularInput.text())
            limit_linear = float(self.LimitLinearInput.text())
            limit_angular = float(self.LimitAngularInput.text())
        except ValueError as e:
            self.ros_signal.emit('logger_label', str(e))
            return

        # Input Validation
        if kp_linear <= 0 or kp_angular <= 0 or limit_linear <= 0 or limit_angular <= 0:
            self.ros_signal.emit('logger_label', 'Invalid input: Gains must be positive values.')
            return

        req = SetFloats.Request()
        req.data = [kp_linear, kp_angular, limit_linear, limit_angular]
        
        if self.GNSSGainsRadioButton.isChecked():
            future = self.set_autopilot_gains_client.call_async(req)
        elif self.ObjArucoGainsRadioButton.isChecked():
            future = self.set_aruco_autopilot_gains_client.call_async(req)
        else:
            self.ros_signal.emit('logger_label', 'Please Select an Autopilot Gain Type')
            return

        future.add_done_callback(self.set_gains_callback)
        self.ros_signal.emit('logger_label', 'Sending Gains...')

    def set_gains_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.ros_signal.emit('logger_label', response.message)
            else:
                self.ros_signal.emit('logger_label', "Failed to send gains")
        except Exception as e:
            self.ros_signal.emit('logger_label', f'Send Gains Service call failed! {e}')
        return

    # def set_aruco_autopilot_gains(self, kp_linear, kp_angular, limit_linear, limit_angular):
    #     self.get_logger().warn('Attempting to set aruco autopilot gains')


    def rover_state_singleton_callback(self, msg):
        self.rover_state_singleton_timepoint = self.get_clock().now().to_msg().sec
        self.RoverStateLat.setText(f'Latitude: {msg.gps.latitude}')
        self.RoverStateLon.setText(f'Longitude: {msg.gps.longitude}')
        self.RoverStateMapYaw.setText(f'Map Yaw: {msg.map_yaw}')
        return

    # GUI Graphics Functions
    def update_leg_subselection(self):
        if self.GNSSRadioButton.isChecked():
            self.leg_type = 'GNSS'
            self.tag_id = 'GPS_only'
            self.legsubselectionStackedWidget.setCurrentIndex(0)
        elif self.ArUcoRadioButton.isChecked():
            self.leg_type = 'ArUco'
            self.tag_id = None
            self.legsubselectionStackedWidget.setCurrentIndex(1)

            # Clear the Object radio buttons for update_tag_selection function to work
            self.BottleRadioButton.setAutoExclusive(False)
            self.BottleRadioButton.setChecked(False)
            self.BottleRadioButton.setAutoExclusive(True)
            self.MalletRadioButton.setAutoExclusive(False)
            self.MalletRadioButton.setChecked(False)
            self.MalletRadioButton.setAutoExclusive(True)

        elif self.ObjectRadioButton.isChecked():
            self.leg_type = 'Object'
            self.tag_id = None
            self.legsubselectionStackedWidget.setCurrentIndex(2)

            # Clear the Aruco radio buttons for update_tag_selection function to work
            for i in range(1, 4):
                getattr(self, f'Tag{i}RadioButton').setAutoExclusive(False)
                getattr(self, f'Tag{i}RadioButton').setChecked(False)
                getattr(self, f'Tag{i}RadioButton').setAutoExclusive(True)
        return

    def update_tag_selection(self):
        if self.Tag1RadioButton.isChecked():
            self.tag_id = '1'
        elif self.Tag2RadioButton.isChecked():
            self.tag_id = '2'
        elif self.Tag3RadioButton.isChecked():
            self.tag_id = '3'
        elif self.BottleRadioButton.isChecked():
            self.tag_id = 'bottle'
        elif self.MalletRadioButton.isChecked():
            self.tag_id = 'mallet'
        else:
            self.tag_id = None
        return
    
    # NOTE: This function is to fix errors caused by differing gui and ros2 threading protocol causing false positives for gui attribute deletion
    def gui_setText(self, name, text):
        if hasattr(self, str(name)):
            getattr(self, str(name)).setText(text)
        else:
            self.get_logger().warn(f'Could not find {name} field of gui')


    #NOTE: depricated until mapviz capability added back
    # This reorders the added waypoints to the optimal order based on path length
    # def plan_order_mapviz_service_call(self):
    #     req = OrderPath.Request() # Path
    #     req.path = self.current_previewed_waypoints

    #     future = self.plan_order_mapviz_client.call_async(req)
    #     self.ros_signal.emit('logger_label', 'Planning order on mapviz...')

    # This clears all previewed waypoints from mapviz
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
        self.ros_signal.emit('logger_label', 'Mapviz Cleared')
    
#NOTE: not used anywhere
# Converts a path from UTM to lat/lon
# def path_to_latlon(path, utm_easting_zero, utm_northing_zero, utm_zone_number, utm_zone_letter):
#     latlon_path = Path()
#     latlon_path.header = path.header
#     for pose in path.poses:
#         lat, lon = utm.to_latlon(pose.pose.position.x + utm_easting_zero, pose.pose.position.y + utm_northing_zero, utm_zone_number, utm_zone_letter)
#         latlon_path.poses.append(
#             PoseStamped(
#                 header=pose.header, 
#                 pose=Pose(
#                     position=Point(x=lat, y=lon, z=0.0),
#                     orientation=pose.pose.orientation
#                 )
#             )
#         )
#     return latlon_path

# Converts a path from lat/lon to UTM
def path_to_utm(path, utm_easting_zero, utm_northing_zero):
    utm_path = Path()
    utm_path.header = path.header
    for pose in path.poses:
        utm_coords = utm.from_latlon(pose.pose.position.x, pose.pose.position.y)
        utm_path.poses.append(
            PoseStamped(
                header=pose.header,
                pose=Pose(
                    position=Point(x=utm_coords[0] - utm_easting_zero, y=utm_coords[1] - utm_northing_zero, z=0.0),
                    orientation=pose.pose.orientation
                )
            )
        )
    return utm_path
            
def gui_ros_spin_thread(node):
    rclpy.spin(node)

def main(args=None):
    # Initialize ROS2
    rclpy.init(args=args)

    # Create QApplication
    gui_QWidget = QApplication(sys.argv)

    # Create ROS Gui
    gui_ros = AutonomyGUI()

    # Create a thread to spin the ROS node
    ros_thread = threading.Thread(target=gui_ros_spin_thread, args=(gui_ros,), daemon=True)
    ros_thread.start()

    # Execute the GUI
    sys.exit(gui_QWidget.exec_())

    rclpy.shutdown()

if __name__ == '__main__':
    main()