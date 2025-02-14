"""
Autonomy GUI

By: Daniel Webb and Gabe Slade - 2025

Notes: 
The QWidget Runs all the time and the ros node is spun in a different thread

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
import numpy as np

# For Mapviz Usage
import yaml
import utm

from std_srvs.srv import SetBool
from std_msgs.msg import Header
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Point
from rover_msgs.srv import AutonomyAbort, AutonomyWaypoint, OrderPath, SetFloat32
from rover_msgs.msg import AutonomyTaskInfo, RoverStateSingleton, RoverState, NavStatus, FiducialData, FiducialTransformArray, ObjectDetections, MobilityAutopilotCommand, MobilityVelocityCommands, MobilityDriveCommand, IWCMotors
from ublox_read_2.msg import PositionVelocityTime #TODO: Uncomment this and get ublox_read_2 working, delete PositionVelocityTime from rover_msgs
from ament_index_python.packages import get_package_share_directory

import threading


class AutonomyGUI(Node, QWidget):
    def __init__(self):
        # Initialize ROS2 node
        Node.__init__(self, 'autonomy_gui')

        # Initialize QWidget
        QWidget.__init__(self)
        # Load the .ui file
        uic.loadUi(os.path.expanduser('~') + '/mars_ws/src/autonomy/autonomy_gui.ui', self)
        self.show()  # Show the GUI

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
        self.ClearWaypointButton.clicked.connect(self.clear_waypoint)

        self.PreviewMapvizButton.clicked.connect(self.preview_waypoint)
        self.PlanOrderButton.clicked.connect(self.plan_order_service_call)
        self.ClearMapvizButton.clicked.connect(self.clear_mapviz)

        self.SetTurnConstantButton.clicked.connect(self.set_turn_constant)
        self.SetSpeedConstantButton.clicked.connect(self.set_speed_constant)

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

        ################# ROS Communication #################

        # Publishers
        self.path_publisher = self.create_publisher(Path, '/mapviz/path', 10)

        # Subscribers
        self.create_subscription(PositionVelocityTime, '/base/PosVelTime', self.base_GPS_info_callback, 10) #GPS info from base station
        self.create_subscription(PositionVelocityTime, '/rover/PosVelTime', self.rover_GPS_info_callback, 10) #GPS info from rover
        self.create_subscription(RoverStateSingleton, '/odometry/rover_state_singleton', self.rover_state_singleton_callback, 10) #Rover GPS and Heading
        self.create_subscription(RoverState, "/rover_status", self.rover_state_callback, 10) #Rover state (speed, direction, navigation state)
        self.create_subscription(NavStatus, '/nav_status', self.rover_nav_status_callback, 10) #Autonomy State machine status
        self.create_subscription(FiducialTransformArray, '/aruco_detect_logi/fiducial_transforms', self.ar_tag_callback, 10) #Aruco Detection
        self.create_subscription(ObjectDetections, '/zed/object_detection', self.obj_detect_callback, 10) #Object Detection
        self.create_subscription(MobilityAutopilotCommand, '/mobility/autopilot_cmds', self.autopilot_cmds_callback, 10) #What mobility/path_manager is publishing
        self.create_subscription(MobilityVelocityCommands, '/mobility/rover_vel_cmds', self.vel_cmds_callback, 10) #What mobility/autopilot_manager is publishing
        self.create_subscription(MobilityDriveCommand, '/mobility/wheel_vel_cmds', self.wheel_vel_cmds_callback, 10) #What mobility/wheel_manager is publishing
        self.create_subscription(IWCMotors, '/mobility/auto_drive_cmds', self.auto_drive_cmds_callback, 1)


        # Services

        # Clients
        self.plan_order_client = self.create_client(OrderPath, '/plan_order')
        self.enable_autonomy_client = self.create_client(SetBool, '/autonomy/enable_autonomy')
        self.send_waypoint_client = self.create_client(AutonomyWaypoint, '/AU_waypoint_service')
        self.clear_waypoint_client = self.create_client(SetBool, '/AU_clear_waypoint_service')
        self.abort_autonomy_client = self.create_client(AutonomyAbort, '/autonomy/abort_autonomy')
        self.set_turn_constant_client = self.create_client(SetFloat32, '/mobility/drive_manager/set_turn_constant')
        self.set_speed_constant_client = self.create_client(SetFloat32, '/mobility/drive_manager/set_speed')

        # Timer to run check if we have recieved information from various sources recently
        self.timepoints_timer = self.create_timer(0.5, self.check_timepoints)
        self.rover_state_singleton_timepoint = None

        ################# Debug Setup #################

        

        ################# Mapviz Communication Setup #################

        # Retrieve Mapviz Location
        self.declare_parameter('location', 'hanksville')
        location = self.get_parameter('location').value

        # Use Location to get the lat and lon corresponding to the mapviz (0, 0) coordinate
        mapviz_params_path = os.path.join(get_package_share_directory('mapviz_tf'), 'params', 'mapviz_params.yaml')
        lat, lon = get_coordinates(mapviz_params_path, location)
        print(f'Lat: {lat}, Lon: {lon}')

        # Convert lat/lon to UTM coordinates
        utm_coords = utm.from_latlon(lat, lon)
        self.utm_easting_zero = utm_coords[0]
        self.utm_northing_zero = utm_coords[1]
        self.utm_zone_number = utm_coords[2]
        self.utm_zone_letter = utm_coords[3]

        # Initialize the current previewed waypoints
        # Stored in lat/lon format
        self.current_previewed_waypoints = Path()


    def check_timepoints(self):
        if self.rover_state_singleton_timepoint is not None:
            if self.get_clock().now().to_msg().sec - self.rover_state_singleton_timepoint > 1:
                self.clear_status()
    
    def clear_status(self):
        self.RoverStateMapYaw.setText('Map Yaw: ...')
        self.RoverStateLat.setText('Latitude: ...')
        self.RoverStateLon.setText('Longitude: ...')
        return

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
        self.BaseSats.setText(f'Sat #: {self.base_numSV}')
        self.BaseDate.setText(f'Date: {base_month}/{base_day}/{base_year}')
        self.BaseTime.setText(f'Time: {base_hour}:{base_min}:{base_sec}')
        self.BaseLat.setText(f'Lat: {round(msg.lla[0], 6)}')
        self.BaseLon.setText(f'Lon: {round(msg.lla[1], 6)}')
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
        self.RoverSats.setText(f'Sat #: {self.rover_numSV}')
        self.RoverDate.setText(f'Date: {rover_month}/{rover_day}/{rover_year}')
        self.RoverTime.setText(f'Time: {rover_hour}:{rover_min}:{rover_sec}')
        self.RoverLat.setText(f'Lat: {round(msg.lla[0], 6)}')
        self.RoverLon.setText(f'Lon: {round(msg.lla[1], 6)}')
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
        #Update previous state and state list
        if self.state_machine_state != None and msg.state != self.state_machine_state:
            self.state_machine_list_string = f'{msg.state}\n' + self.state_machine_list_string
            self.PreviousStatesList.setText(self.state_machine_list_string) 

            self.prev_state_machine_state = self.state_machine_state
            self.PreviousMainStateDisplay.setText(self.prev_state_machine_state)

        #Show the first state on the previous state column
        if self.state_machine_state == None:
            self.state_machine_list_string = f'{msg.state}\n'
            self.PreviousStatesList.setText(self.state_machine_list_string)

        #Update current state and autonomous enable
        self.state_machine_state = msg.state
        autonomous_enable = msg.auto_enable
        if autonomous_enable:
            self.autonomous_enable = 'Enabled'
        else:
            self.autonomous_enable = 'Disabled'

        self.CurrentMainStateDisplay.setText(self.state_machine_state)
        
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
            
            self.ArucoStatus.setText(aruco_text)
        return
    
    def obj_detect_callback(self, msg):
        obj_name = None
        objects_string = ''
        for obj in msg.objects:
            if obj.label == 1:
                obj_name = 'Bottle'
            elif obj.label == 2:
                obj_name = 'Hammer'
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

        self.ObjStatus.setText(objects_string)
        return
    
    ################# Callbacks for Mobility #################
    def autopilot_cmds_callback(self, msg):
        self.autopilot_cmds_msg = msg
        self.setAutopilotString(self.autopilot_cmds_msg)        
        return

    #helper function for autopilot_cmds_callback and vel_cmds_callback
    def setAutopilotString(self, msg):
        if self.course_heading_error is None:
            autopilot_cmds_string = f'Dist to target: {round(msg.distance_to_target, 2)}m, cw from N: {round(np.rad2deg(msg.course_angle), 2)}°'
        else:
            autopilot_cmds_string = f'Dist to target: {round(msg.distance_to_target, 2)}m, cw from N: {round(np.rad2deg(msg.course_angle), 2)}°, Heading Err: {round(np.rad2deg(self.course_heading_error), 2)}°'
        self.AutopilotCmds.setText(autopilot_cmds_string)
        return
    
    def vel_cmds_callback(self, msg):
        self.course_heading_error = msg.course_heading_error
        vel_cmds_string = f'Lin Vel: {round(msg.u_cmd, 2)}, Ang Vel: {round(msg.omega_cmd, 2)}'
        self.VelocityCmds.setText(vel_cmds_string)

        # If we have a course heading error, update the autopilot string
        if self.autopilot_cmds_msg is not None:
            self.setAutopilotString(self.autopilot_cmds_msg)

        return

    def wheel_vel_cmds_callback(self, msg):
        wheel_vel_cmds_string = f'LW Speed: {round(msg.lw, 2)}, RW Speed: {round(msg.rw, 2)}'
        self.WheelVelocityCmds.setText(wheel_vel_cmds_string)
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

        
        self.IWCCmds.setText(IWC_cmd_string)
        
        return

    # Callback functions for buttons
    def enable_autonomy(self):
        req = SetBool.Request()
        req.data = True
        future = self.enable_autonomy_client.call_async(req)
        self.error_label.setText('Enabling Autonomy...')
        # future.add_done_callback(self.future_callback)

    def disable_autonomy(self):
        req = SetBool.Request()
        req.data = False
        future = self.enable_autonomy_client.call_async(req)
        self.error_label.setText('Disabling Autonomy...')
        # future.add_done_callback(self.future_callback)

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
        
        self.error_label.setText('Waypoint Sent for Preview')

    def plan_order_service_call(self):
        req = OrderPath.Request() # Path
        req.path = self.current_previewed_waypoints

        future = self.plan_order_client.call_async(req)
        self.error_label.setText('Planning Order...')
        # future.add_done_callback(self.future_callback)

        # TODO: need a response from this future... but the ros node is spinning in another thread
        # response = future.result()

        # self.current_previewed_waypoints = response.path
        # # self.current_previewed_waypoints.header.frame_id = 'map'
        # self.path_publisher.publish(
        #     path_to_utm(self.current_previewed_waypoints, 
        #                 self.utm_easting_zero, 
        #                 self.utm_northing_zero)
        #     )

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
            self.get_logger().info('Sending Waypoint...')
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

        # Send the Waypoint
        self.error_label.setText('Sending Waypoint')
        future = self.send_waypoint_client.call_async(req)
        # future.add_done_callback(self.future_callback)
        return

    def clear_waypoint(self):
        req = SetBool.Request()
        req.data = True
        future = self.clear_waypoint_client.call_async(req)
        self.error_label.setText('Removing Last Waypoint')
        future.add_done_callback(self.clear_waypoint_callback)

    def clear_waypoint_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.error_label.setText(response.message)
            else:
                self.error_label.setText("Failed to remove waypoint")
        except Exception as e:
            self.error_label.setText(f'Remove Waypoint Service call failed!')

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

        # Send the Abort Request
        future = self.abort_autonomy_client.call_async(req)
        self.error_label.setText('Attempting Abort')
        # future.add_done_callback(self.future_callback)

    def set_turn_constant(self):
        req = SetFloat32.Request()
        req.data = float(self.TurnConstantInput.text())
        future = self.set_turn_constant_client.call_async(req)
        self.error_label.setText('Sending Turn Constant...')
        future.add_done_callback(self.set_turn_constant_callback)

    def set_turn_constant_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.error_label.setText(response.message)
            else:
                self.error_label.setText("Failed to send turn constant")
        except Exception as e:
            self.error_label.setText(f'Send Turn Constant Service call failed!')

    def set_speed_constant(self):
        req = SetFloat32.Request()
        req.data = float(self.SpeedConstantInput.text())
        future = self.set_speed_constant_client.call_async(req)
        self.error_label.setText('Sending Speed Constant...')
        future.add_done_callback(self.set_speed_constant_callback)

    def set_speed_constant_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.error_label.setText(response.message)
            else:
                self.error_label.setText("Failed! Must be in range (0-10)")
        except Exception as e:
            self.error_label.setText(f'Send Speed Constant Service call failed!')

    def rover_state_singleton_callback(self, msg):
        self.rover_state_singleton_timepoint = self.get_clock().now().to_msg().sec
        self.RoverStateLat.setText(f'Latitude: {msg.gps.latitude}')
        self.RoverStateLon.setText(f'Longitude: {msg.gps.longitude}')
        self.RoverStateMapYaw.setText(f'Map Yaw: {msg.map_yaw}')
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
    
    # Service calls generic callback
    def future_callback(future):
        # try:
        #     response = future.result()
        #     if response.success:
        #         self.error_label.setText(success_msg)
        #     else:
        #         self.error_label.setText(error_msg)
        # except Exception as e:
        #     self.error_label.setText(f'Service call failed: {str(e)}')
        pass

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
    
# Converts a path from UTM to lat/lon
def path_to_latlon(path, utm_easting_zero, utm_northing_zero, utm_zone_number, utm_zone_letter):
    latlon_path = Path()
    latlon_path.header = path.header
    for pose in path.poses:
        lat, lon = utm.to_latlon(pose.pose.position.x + utm_easting_zero, pose.pose.position.y + utm_northing_zero, utm_zone_number, utm_zone_letter)
        latlon_path.poses.append(
            PoseStamped(
                header=pose.header, 
                pose=Pose(
                    position=Point(x=lat, y=lon, z=0.0),
                    orientation=pose.pose.orientation
                )
            )
        )
    return latlon_path

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

    Popen("pkill gst", shell=True, preexec_fn=os.setsid, stderr=PIPE)
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