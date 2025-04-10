#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rover_msgs.msg import AutonomyTaskInfo, RoverStateSingleton, NavState, RoverState, FiducialData, FiducialTransformArray, ObjectDetections
from rover_msgs.srv import SetFloat32, AutonomyAbort, AutonomyWaypoint
from std_srvs.srv import SetBool
from std_msgs.msg import String
from autonomy.drive_controller_api import DriveControllerAPI
from autonomy.GPSTools import GPSTools, GPSCoordinate
from enum import Enum
import numpy as np
import time
from collections import deque

class State(Enum):
    MANUAL = "MANUAL"
    SEARCH_FOR_WRONG_TAG = "SEARCH_FOR_WRONG_TAG"
    START_POINT_NAVIGATION = "START_POINT_NAVIGATION"
    WAYPOINT_NAVIGATION = "WAYPOINT_NAVIGATION"
    POINT_NAVIGATION = "POINT_NAVIGATION"
    START_SPIN_SEARCH = "START_SPIN_SEARCH"
    SPIN_SEARCH = "SPIN_SEARCH"
    START_HEX_SEARCH = "START_HEX_SEARCH"
    HEX_SEARCH = "HEX_SEARCH"
    START_OBJECT_HEX_SEARCH = "START_OBJECT_HEX_SEARCH"
    ARUCO_NAVIGATE = "ARUCO_NAVIGATE"
    OBJECT_NAVIGATE = "OBJECT_NAVIGATE"
    ARUCO_GATE_NAVIGATION = "ARUCO_GATE_NAVIGATION"
    ARUCO_GATE_ORIENTATION = "ARUCO_GATE_ORIENTATION"
    ARUCO_GATE_APPROACH = "ARUCO_GATE_APPROACH"
    ARUCO_GATE_PAST = "ARUCO_GATE_PAST"
    TASK_COMPLETE = "TASK_COMPLETE"
    START_ABORT_STATE = "START_ABORT_STATE"
    ABORT_STATE = "ABORT_STATE"

NAVIGATION_STATES = [State.START_POINT_NAVIGATION, State.WAYPOINT_NAVIGATION, State.POINT_NAVIGATION, State.START_SPIN_SEARCH, State.SPIN_SEARCH, State.START_HEX_SEARCH ,State.HEX_SEARCH, State.START_OBJECT_HEX_SEARCH, State.ARUCO_NAVIGATE, State.OBJECT_NAVIGATE, State.ARUCO_GATE_NAVIGATION, State.ARUCO_GATE_ORIENTATION, State.ARUCO_GATE_APPROACH, State.ARUCO_GATE_PAST]

class TagID(Enum):
    AR_TAG_1 = 1
    AR_TAG_2 = 2
    AR_TAG_3 = 3
    BOTTLE = 4
    MALLET = 5
    GPS_ONLY = 6

AR_ENABLE_MSG = 'aruco detections - enabled'
AR_DISABLE_MSG = 'aruco detections - disabled'
OBJ_ENABLE_MSG = ''     # TODO
OBJ_DISABLE_MSG = ''    # TODO

class AutonomyStateMachine(Node):

    def __init__(self):
        super().__init__('state_machine')

        # TODO add conditionals to see if in verbose state
        self.verbose = False       # If you want additional log output to debug

        # Create a timer to call `state_loop` every 0.1 seconds (10 Hz)
        self.create_timer(0.1, self.state_loop)

        # Subscribers
        self.create_subscription(RoverStateSingleton, '/odometry/rover_state_singleton', self.rover_state_singleton_callback, 10)
        self.create_subscription(FiducialTransformArray, '/aruco_detect_logi/fiducial_transforms', self.ar_tag_callback, 10)
        self.create_subscription(ObjectDetections, '/zed/object_detection', self.obj_detect_callback, 10)

        # Publishers
        self.aruco_pose_pub = self.create_publisher(FiducialData, '/autonomy/aruco_pose', 10)
        self.nav_state_pub = self.create_publisher(NavState, '/nav_state', 10) # Navigation State
        self.rover_state_pub = self.create_publisher(RoverState, '/rover_state', 10) # State Machine State
        self.debug_pub = self.create_publisher(String, '/state_machine_debug', 10)

        # Services
        self.srv_switch_auto = self.create_service(SetBool, '/autonomy/enable_autonomy', self.enable)
        self.srv_switch_abort = self.create_service(AutonomyAbort, '/autonomy/abort_autonomy', self.abort)
        self.srv_receive_waypoint = self.create_service(AutonomyWaypoint, '/AU_waypoint_service', self.receive_waypoint)
        self.srv_clear_waypoint = self.create_service(SetBool, '/AU_clear_waypoint_service', self.clear_waypoint)

        # Clients
        self.object_detect_client = self.create_client(SetBool, '/toggle_object_detection')
        self.aruco_detect_client = self.create_client(SetBool, '/enable_detections')
        self.set_speed_client = self.create_client(SetFloat32, '/mobility/drive_manager/set_speed')
        self.path_manager_client = self.create_client(SetBool, '/mobility/path_manager/enabled')
        self.autopilot_manager_client = self.create_client(SetBool, '/mobility/autopilot_manager/enabled')
        self.drive_manager_client = self.create_client(SetBool, '/mobility/drive_manager/enabled')
        self.wheel_manager_client = self.create_client(SetBool, '/mobility/wheel_manager/enabled')
        self.aruco_manager_client = self.create_client(SetBool, '/mobility/aruco_autopilot_manager/enabled')

        # Client variables
        self.max_retries = 5
        self.retry_count = 0

        # Declare Parameters
        self.declare_parameter('path_waypoint_distance_tolerance', 4.0)
        self.declare_parameter('distance_tolerance', 1.0)
        self.declare_parameter('obj_distance_tolerance', 1.5) # TODO: Tune & in the yaml
        self.declare_parameter('aruco_distance_tolerance', 2.0)
        self.declare_parameter('abort_distance_tolerance', 2.0)
        self.declare_parameter('hex_search_radius', 17.0)
        self.declare_parameter('navigate_speed', 10.0)
        self.declare_parameter('aruco_speed', 0.3)
        self.declare_parameter('spin_speed', 30.0)
        self.declare_parameter('object_alpha_lpf', 0.5)
        self.declare_parameter('obj_enable_distance', 30.0) #TODO: tune distance from GNSS coordinate that object deteciton is enabled
        self.declare_parameter('aruco_enable_distance', 30.0) #TODO: tune distance from GNSS coordinate that aruco deteciton is enabled
        self.declare_parameter('aruco_alpha_lpf', 0.5)
        self.declare_parameter('spin_step_size', 0.6981)
        self.declare_parameter('spin_delay_time', 1.2)
        self.declare_parameter('wrong_aruco_backup_distance', 7.0)
        self.declare_parameter('wrong_aruco_backup_spin_speed', 15.0)
        self.declare_parameter('hex_seach_angle_difference', 50.0)
        self.declare_parameter('object_speed', 0.3)
        

        # Get Parameters
        self.path_waypoint_dist_tolerance = self.get_parameter('path_waypoint_distance_tolerance').get_parameter_value().double_value #distance tolerance for switching to a different intermediate points in a path planned path
        self.dist_tolerance = self.get_parameter('distance_tolerance').get_parameter_value().double_value #tolerance for stopping at a GNSS coordinate (whether for GPS only legs, or for initial aruco/object GPS coordinates)
        self.obj_dist_tolerance = self.get_parameter('obj_distance_tolerance').get_parameter_value().double_value #distance that the rover stops from an object (bottle or hammer) while navigating to an object
        self.abort_dist_tolerance = self.get_parameter('abort_distance_tolerance').get_parameter_value().double_value #distance tolerance for autonomously returning to an abort point
        self.aruco_dist_tolerance = self.get_parameter('aruco_distance_tolerance').get_parameter_value().double_value #distance that the rover stops from an aruco tag while navigating to an aruco tag
        self.hex_search_radius = self.get_parameter('hex_search_radius').get_parameter_value().double_value #initial radius for hex search for aruco legs (object legs start with 1/3 of this parameter, and increment over time)
        self.navigate_speed = self.get_parameter('navigate_speed').get_parameter_value().double_value #speed that the rover moves while navigating to a GNSS coordinate
        self.object_speed = self.get_parameter('object_speed').get_parameter_value().double_value #speed that the rover moves while navigating to an object after having seen it
        self.aruco_speed = self.get_parameter('aruco_speed').get_parameter_value().double_value #speed that the rover moves while navigating to an aruco
        self.spin_speed = self.get_parameter('spin_speed').get_parameter_value().double_value #angular speed during spin search
        self.obj_alpha_lpf = self.get_parameter('object_alpha_lpf').get_parameter_value().double_value #alpha value for the low pass filter for the angle and distance to an object after the rover starts seeing the object
        self.obj_enable_distance = self.get_parameter('obj_enable_distance').get_parameter_value().double_value # object detection gets enabled only when within a certain distance of the coordinate to conserve computational resources
        self.aruco_alpha_lpf = self.get_parameter('aruco_alpha_lpf').get_parameter_value().double_value #alpha value for the low pass filter for the angle and distance to an aruco tag after the rover starts seeing the aruco tag
        self.spin_step_size = self.get_parameter('spin_step_size').get_parameter_value().double_value # angle in radians that the rover turns in spin search, after which the rover stops, looks for a tag/object, and then continues spinning
        self.spin_delay_time = self.get_parameter('spin_delay_time').get_parameter_value().double_value # time that the rover waits after spinning in spin search looking for tags/objects
        self.wrong_aruco_backup_distance = self.get_parameter('wrong_aruco_backup_distance').get_parameter_value().double_value # distance that the rover backups up upon starting a new leg, after having just finished an aruco tag leg (so that the rover doesn't run into the aruco stand)
        self.wrong_aruco_backup_spin_speed = self.get_parameter('wrong_aruco_backup_spin_speed').get_parameter_value().double_value # speed that the rover spins at when backing up after starting a new leg, after having just finished an aruco tag leg
        self.hex_seach_angle_difference = self.get_parameter('hex_seach_angle_difference').get_parameter_value().double_value # angle in degrees between vertex points of the hex search
        self.aruco_enable_distance = self.get_parameter('aruco_enable_distance').get_parameter_value().double_value # aruco detection gets enabled only when within a certain distance of the target coordinate to conserve computational resources

        #Initialize variables
        self.nav_state = NavState()
        self.drive_controller = DriveControllerAPI(self)
        self.state = State.MANUAL
        self.enabled = False
        self.aruco_tag_distance = None
        self.aruco_tag_angle = None
        self.correct_aruco_tag_found = False
        self.wrong_aruco_tag_found = False
        self.obj_distance = None
        self.obj_angle = None
        self.correct_obj_found = False
        self.ar_callback_see_time = 0
        self.known_objects = {}
        self.target_latitude = 0
        self.target_longitude = 0
        self.target_point = GPSCoordinate(self.target_latitude, self.target_longitude, 0)
        self.curr_latitude = 0
        self.curr_longitude = 0
        self.curr_elevation = 0
        self.curr_heading = 0
        self.current_point = GPSCoordinate(self.curr_latitude, self.curr_longitude, self.curr_elevation)  
        self.tag_id = TagID.GPS_ONLY
        self.a_task_complete = False
        self.aruco_detect_enabled = False
        self.obj_detect_enabled = False
        self.prev_tag_id = TagID.GPS_ONLY
        self.dist_to_target = 1000          # Abitrary High number, Will be updated in point navigation

        # Data structure to hold all of the waypoints at a time
        self.waypoints: deque[AutonomyTaskInfo] = deque()
        self.last_waypoint: AutonomyTaskInfo = None

        self.backup_timer = None
        self.backup_duration = 4.0  # Tune this number 

        self.get_logger().info('Autonomy State Machine initialized')

    def set_task_callback(self, task_info: AutonomyTaskInfo):

        self.target_latitude = task_info.latitude
        self.target_longitude = task_info.longitude
        self.target_point = GPSCoordinate(self.target_latitude, self.target_longitude, 0)

        if task_info.tag_id == 'GPS_only':
            self.tag_id = TagID.GPS_ONLY
        elif task_info.tag_id == '1':
            self.tag_id = TagID.AR_TAG_1
        elif task_info.tag_id == '2':
            self.tag_id = TagID.AR_TAG_2
        elif task_info.tag_id == '3':
            self.tag_id = TagID.AR_TAG_3
        elif task_info.tag_id == 'bottle':
            self.tag_id = TagID.BOTTLE
        elif task_info.tag_id == 'mallet':
            self.tag_id = TagID.MALLET
    
    # Service callback to receive waypoints in the form messages
    def receive_waypoint(self, request: AutonomyWaypoint.Request, response: AutonomyWaypoint.Response) -> AutonomyWaypoint.Response:
        # Clear current waypoint before receiving a new one
        while len(self.waypoints) > 0:
            self.waypoints.pop()

        waypoints = request.task_list
        for wp in waypoints:
            self.waypoints.append(wp) # Append new waypoints to the deque

        self.get_logger().info(f'Waypoints: {self.waypoints}')

        # Transition the state machine to the next state if it's ready to begin
        if len(self.waypoints) > 0:
            self.get_logger().info("Waypoint(s) added, Waiting for autonomy enable")
        else:
            self.get_logger().warn("No waypoints received, staying idle")

        response.success = True
        response.message = 'Adding waypoints was successful'
        self.get_logger().info(f"Response: success={response.success}, message='{response.message}'")
        return response

    def set_current_task(self): # NOTE: Depreciate?
        '''
        Sets current task to be the next one in the deque. Note that this also assumes that the TASK_COMPLETE task has popped
        the first waypoint off the front of the deque.
        '''
        current_task: AutonomyTaskInfo = self.waypoints[0]
        self.set_task_callback(current_task)

        # TODO: Do I want to make this be the current location or the first waypoint?
        if self.last_waypoint is None:
            self.last_waypoint = current_task

    def clear_waypoint(self, request: SetBool.Request, response: SetBool.Response):
        if len(self.waypoints) > 0:
            while len(self.waypoints) > 0:
                self.waypoints.pop()
            self.get_logger().info('Waypoint removed')
            response.success = True
            response.message = f'Waypoint removed successfully'
        else:
            self.get_logger().warn('No waypoint to remove')
            response.success = False
            response.message = 'No waypoint to remove'

        return response

    def rover_state_singleton_callback(self, msg: RoverStateSingleton):
        self.curr_latitude = msg.gps.latitude
        self.curr_longitude = msg.gps.longitude
        self.curr_elevation = msg.gps.altitude
        # TODO: Get our filtered GPS POSITION INSTEAD OF RAW
        self.current_point = GPSCoordinate(self.curr_latitude, self.curr_longitude, self.curr_elevation)
        self.curr_heading = np.deg2rad(msg.map_yaw)

    def toggle_object_detection(self, data):
        if self.object_detect_client.service_is_ready():  # Check if service is available
            self.get_logger().info('Sending object detection request...')
            self.send_request(data, self.object_detect_client)
            self.retry_count = 0  # Reset retry count
        else:
            if self.retry_count < self.max_retries:
                self.get_logger().warn(f'Service not available. Retrying... ({self.retry_count + 1}/{self.max_retries})')
                self.retry_count += 1
                self.create_timer(1.0, lambda: self.toggle_object_detection(data))
            else:
                self.get_logger().error('Object detection service not available after maximum retries. Giving up.')

    def toggle_aruco_detection(self, data):
        if self.aruco_detect_client.service_is_ready():
            self.get_logger().info('Sending aruco detection request...')
            self.send_request(data, self.aruco_detect_client)
            self.retry_count = 0 # Reset retry count
        else:
            if self.retry_count < self.max_retries:
                self.get_logger().warn(f'Service not available. Retrying... ({self.retry_count + 1}/{self.max_retries})')
                self.retry_count += 1
                self.create_timer(1.0, lambda: self.toggle_aruco_detection(data))
            else:
                self.get_logger().error('Aruco detection service not available after maximum retries. Giving up.')

    # Generic service call method
    def send_request(self, data, client):
        # Create and send a request
        request = SetBool.Request()
        request.data = data
        future = client.call_async(request)
        future.add_done_callback(self.handle_response)

    # Callback for handling the service call function (send_request)
    def handle_response(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Response: Success={response.success}, Message="{response.message}"')
            if response.message == AR_ENABLE_MSG:
                self.aruco_detect_enabled = True
            elif response.message == AR_DISABLE_MSG:
                self.aruco_detect_enabled = False
            elif response.message == OBJ_ENABLE_MSG:
                self.obj_detect_enabled = True
            elif response.message == OBJ_DISABLE_MSG:
                self.obj_detect_enabled = False
            
        except Exception as e:
            self.get_logger().error(f'Failed to call service: {str(e)}')

    def obj_detect_callback(self, msg: ObjectDetections):
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        is_recent = lambda obj_ts: timestamp - obj_ts <= 1.0

        # Remove objects that haven't been seen in the last second
        self.known_objects = {k: v for k, v in self.known_objects.items() if is_recent(v[-1])}

        correct_label = 0 
        if self.tag_id == TagID.BOTTLE:
            correct_label = 1
        
        found = False
        for obj in msg.objects:
            msg = String()
            msg.data = f"Object: {obj.label}, {obj.confidence}"
            self.debug_pub.publish(msg)
            if obj.label != correct_label or obj.confidence < 0.75:
                continue

            if obj.id in self.known_objects:
                if is_recent(self.known_objects[obj.id][-1]):
                    self.known_objects[obj.id].append(timestamp)

                if len(self.known_objects[obj.id]) < 5:
                    msg = String()
                    msg.data = f"Num Detections: {len(self.known_objects[obj.id])}"
                    self.debug_pub.publish(msg)
                    continue

                # Low-pass filter the distance and heading information
                # For the ZED X is forward, Y is left, Z is up. Positive angle is counterclockwise from x-axis. All in meters.
                obj_dist = np.sqrt((obj.y) ** 2 + (obj.x) ** 2)
                obj_ang = -np.arctan(obj.y / obj.x)
                if self.obj_distance is None:
                    self.obj_distance = obj_dist
                    self.obj_angle = obj_ang
                else:
                    # Low pass filter the distance and heading information
                    self.obj_distance = self.obj_distance * self.obj_alpha_lpf + obj_dist * (1 - self.obj_alpha_lpf)
                    self.obj_angle = self.obj_angle * self.obj_alpha_lpf + obj_ang * (1 - self.obj_alpha_lpf)
                self.correct_obj_found = True
                if found:
                    self.get_logger().info("Found a duplicate object, taking last one")
                else:
                    self.get_logger().info(f"Found object for 15 frames: {obj}")
                    found = True
            else:
                self.known_objects[obj.id] = [timestamp]


    def ar_tag_callback(self, msg: FiducialTransformArray):
        if len(msg.transforms) == 1: 

            #For the webcam giving us this data, X is to the right, Y is down, Z is forward. All in meters. we want a positive angle to be counterclockwise from the z-axis
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

            #Calculate aruco GPS coordinates to be used if we have not seen the tag within the last second (see elif statement below)
            self.current_aruco_point = GPSTools.heading_distance_to_lat_lon(
                self.current_point, 
                np.rad2deg(self.curr_heading + self.aruco_tag_angle), 
                self.aruco_tag_distance
            )

            # TODO: Do we need to publish this data? What is using this data?
            # self.aruco_pose = FiducialData()
            # self.aruco_pose.angle_offset = self.aruco_tag_angle
            # self.aruco_pose.dist_to_fiducial = self.aruco_tag_distance
            # self.aruco_pose_pub.publish(self.aruco_pose)

            self.ar_callback_see_time = time.time()

            if msg.transforms[0].fiducial_id == self.tag_id.value:
                self.get_logger().info(f"Recieved Correct tag: {self.tag_id.value}", throttle_duration_sec=3.0) #correct Tag found, navigate using angle and distance
                self.correct_aruco_tag_found = True
                self.wrong_aruco_tag_found = False
            else:
                self.get_logger().info(f"Recieved  INCORRECT tag. tagID: {msg.transforms[0].fiducial_id}, Correct id: {self.tag_id.value}",throttle_duration_sec=3.0)
                self.correct_aruco_tag_found = False
                self.wrong_aruco_tag_found = True
        elif time.time() - self.ar_callback_see_time > 1: #If we have not seen the tag within the last second, we will use the last known position
            self.correct_aruco_tag_found = False
            self.wrong_aruco_tag_found = False
            # self.both_aruco_tags_found = False

    def enable(self, request: SetBool.Request, response: SetBool.Response):
        self.enabled = request.data
        
        if self.enabled:
            if (len(self.waypoints) == 0):
                #No waypoints
                self.get_logger().warn("No waypoints available, cannot enable autonomy.")
                self.enabled = False
                response.success = False
                response.message = "No waypoints available."
                return response
            else:
                # Set the first task, and get state loop ready
                self.set_current_task()
                self.get_logger().info("Autonomy state machine is enabled!")
                self.state = State.SEARCH_FOR_WRONG_TAG
        else:
            self.get_logger().info("Autonomy state machine is disabled!")
        
        response.success = True
        response.message = f"Autonomy state machine is {self.enabled}!"
        return response
    
    def abort(self, request: AutonomyAbort.Request, response: SetBool.Response): 
        self.get_logger().info('in abort')
        self.abort_status = request.abort_status
        self.abort_lat = request.lat
        self.abort_lon = request.lon
        self.abort_point = GPSCoordinate(self.abort_lat, self.abort_lon, 0)


        if self.abort_status:
            self.get_logger().info("Aborting...")
            self.state = State.START_ABORT_STATE
        else:
            self.get_logger().info("Cancelling abort!")
            self.state = State.MANUAL

        response.success = True
        response.message = f"Abort is {self.abort_status}!"
        return response
    
    def wrap(self, chi_1, chi_2):
        while chi_1 - chi_2 > np.pi:
            chi_1 -= 2.0 * np.pi
        while chi_1 - chi_2 < -np.pi:
            chi_1 += 2.0 * np.pi
        return chi_1

    def stop_backup(self):
        self.drive_controller.issue_drive_cmd(0.0, 0.0)
        self.state = State.START_POINT_NAVIGATION

        # Destroy the timer and reset it
        if self.backup_timer is not None:
            self.backup_timer.cancel()
            self.backup_timer = None


    # This is a service that adjusts the drive manager's max speed, different speeds should be used for local navigation vs GNSS navigation
    def set_speed(self, speed):
        # Speed is from 0 to 10
        req = SetFloat32.Request()
        req.data = float(speed)
        if self.set_speed_client.service_is_ready():
            future = self.set_speed_client.call_async(req)
        else:
            self.get_logger().warn("Set speed service is not available.")
        return

    def check_detections(self):
        # Enable object detection and aruco detection:
            # Only when actively navigating
        if self.state in NAVIGATION_STATES and self.enabled:
            #Toggle object detection or aruco detection if within a certain distance of the target point (saves computation time)
            if self.tag_id in [TagID.MALLET, TagID.BOTTLE] and self.dist_to_target < self.obj_enable_distance:
                if not self.obj_detect_enabled:
                    self.toggle_object_detection(True)
                    self.get_logger().info(f"Enabling Aruco - State is: {self.state.value}, tag_id is: {self.tag_id.value}")
            elif self.tag_id in [TagID.AR_TAG_1, TagID.AR_TAG_2, TagID.AR_TAG_3] and self.dist_to_target < self.aruco_enable_distance:
                if not self.aruco_detect_enabled:
                    self.toggle_aruco_detection(True)
                    self.get_logger().info(f"Enabling Aruco - State is: {self.state.value}, tag_id is: {self.tag_id.value}")

        else:
            if self.aruco_detect_enabled:
                self.toggle_aruco_detection(False)
                self.get_logger().info(f"Disabling Aruco - State is: {self.state.value}, tag_id is: {self.tag_id.value}")
            if self.obj_detect_enabled:
                self.toggle_object_detection(False)
                self.get_logger().info(f"Disabling Object Detection - State is: {self.state.value}, tag_id is: {self.tag_id.value}")


    def reset_state_variables(self):
        # Reset the state machine variables, ONLY if task is complete
        self.correct_aruco_tag_found = False
        self.correct_obj_found = False
        self.obj_distance = None
        self.obj_angle = None
        self.prev_tag_id = self.tag_id
        self.dist_to_target = 1000          # Arbitrary High number, will be updated in navigation


    def state_loop(self):
        self.get_logger().info(f"State is: {self.state.value}", throttle_duration_sec=10)
        
        if self.enabled:

            if self.state == State.MANUAL:
                if self.a_task_complete:
                    #Keep the LED Green by keeping it in arrival state if a task has been completed
                    self.nav_state.navigation_state = NavState.ARRIVAL_STATE

                else:
                    self.nav_state.navigation_state = NavState.TELEOPERATION_STATE

                # Reset the state machine variables
                self.correct_aruco_tag_found = False
                self.correct_obj_found = False
                self.obj_distance = None
                self.obj_angle = None

            #This SEARCH_FOR_WRONG_STATE state is used to ensure that after finishing one aruco tag task, the rover will
            #backup if the rover sees the wrong tag, to ensure it does not run into the aruco stand before starting the next task
            elif self.state == State.SEARCH_FOR_WRONG_TAG: 
                self.nav_state.navigation_state = NavState.AUTONOMOUS_STATE
                if self.prev_tag_id in [TagID.AR_TAG_1, TagID.AR_TAG_2, TagID.AR_TAG_3]:    # instead of enabling aruco, just check if previous tag was an AR and backup anyways

                    #Find the angle to our new target point
                    target_latitude = self.waypoints[0].latitude
                    target_longitude = self.waypoints[0].longitude
                    target_point = GPSCoordinate(target_latitude, target_longitude, 0)
                    chi_rad, chi_deg = GPSTools.heading_between_lat_lon(self.current_point, target_point)
                    des_heading = self.wrap(chi_rad - self.curr_heading, 0)
                    linear_vel = -2.0
                    angular_vel = -self.wrong_aruco_backup_spin_speed if des_heading < 0 else self.wrong_aruco_backup_spin_speed

                    # Start backup maneuver with timer
                    if self.backup_timer is None:
                        self.drive_controller.issue_drive_cmd(linear_vel, angular_vel)
                        self.backup_timer = self.create_timer(self.backup_duration, self.stop_backup)

                else:
                    self.drive_controller.issue_drive_cmd(0.0, 0.0)
                    self.state = State.START_POINT_NAVIGATION

            elif self.state == State.START_POINT_NAVIGATION:
                self.nav_state.navigation_state = NavState.AUTONOMOUS_STATE
                self.get_logger().info("Starting commands")
                self.set_speed(self.navigate_speed)
                self.get_logger().info(f"Number of waypoints {len(self.waypoints)}")
                self.target_latitude = self.waypoints[0].latitude
                self.target_longitude = self.waypoints[0].longitude
                self.target_point = GPSCoordinate(self.target_latitude, self.target_longitude, 0)
                self.drive_controller.issue_path_cmd(self.target_latitude, self.target_longitude)
                self.state = State.WAYPOINT_NAVIGATION

            elif self.state == State.WAYPOINT_NAVIGATION:
                self.nav_state.navigation_state = NavState.AUTONOMOUS_STATE
                if GPSTools.distance_between_lat_lon(self.current_point, self.target_point) < self.path_waypoint_dist_tolerance:
                    if len(self.waypoints) > 1:
                        self.waypoints.popleft()
                        self.get_logger().info("Popped intermediate waypoint!")
                        self.state = State.START_POINT_NAVIGATION
                    else:
                        self.get_logger().info("Approaching coordinate")
                        self.state = State.POINT_NAVIGATION
                if self.correct_aruco_tag_found:
                    self.state = State.ARUCO_NAVIGATE
                elif self.correct_obj_found:
                    self.state = State.OBJECT_NAVIGATE


            elif self.state == State.POINT_NAVIGATION:
                self.nav_state.navigation_state = NavState.AUTONOMOUS_STATE
                self.dist_to_target = GPSTools.distance_between_lat_lon(self.current_point, self.target_point)

                #Toggle object detection or aruco detection if within a certain distance of the target point (saves computation time)
                if self.tag_id in [TagID.MALLET, TagID.BOTTLE] and self.dist_to_target < self.obj_enable_distance:
                    if not self.obj_detect_enabled:
                        self.toggle_object_detection(True)
                elif self.tag_id in [TagID.AR_TAG_1, TagID.AR_TAG_2, TagID.AR_TAG_3] and self.dist_to_target < self.aruco_enable_distance:
                    if not self.aruco_detect_enabled:
                        self.toggle_aruco_detection(True)
                
                if self.dist_to_target < self.dist_tolerance:
                    if self.tag_id == TagID.GPS_ONLY:
                        self.get_logger().info('GPS Task is complete!')
                        self.state = State.TASK_COMPLETE
                        self.drive_controller.stop()
                    else:
                        self.state = State.START_SPIN_SEARCH
                        self.hex_search_point_num = 0
                        self.hex_center_point = self.current_point
                        self.hex_radius_factor = 1/3 if self.tag_id in [TagID.MALLET, TagID.BOTTLE] else 1
                if self.correct_aruco_tag_found:
                    self.state = State.ARUCO_NAVIGATE
                elif self.correct_obj_found:
                    self.state = State.OBJECT_NAVIGATE

            elif self.state == State.START_SPIN_SEARCH:
                self.nav_state.navigation_state = NavState.AUTONOMOUS_STATE
                self.spin_start_heading = self.wrap(self.curr_heading, 0)
                self.spin_stop = False
                self.spin_target_angle = self.wrap(self.curr_heading + self.spin_step_size, 0)
                self.get_logger().info(f"target: {self.spin_target_angle}")
                self.drive_controller.issue_drive_cmd(0.0, self.spin_speed)
                self.state = State.SPIN_SEARCH

            elif self.state == State.SPIN_SEARCH:
                self.nav_state.navigation_state = NavState.AUTONOMOUS_STATE
                msg = String() # For Debugging TODO: remove
                if self.spin_stop:
                    msg.data = "aruco spin Stopping"
                    # If the rover has spun self.spin_step_size and is stopped, wait for self.spin_delay_time seconds to look for a tag
                    if time.time() - self.spin_stop_time > self.spin_delay_time:
                        self.spin_stop = False
                        self.spin_target_angle = self.wrap(self.spin_target_angle + self.spin_step_size, 0)
                        self.drive_controller.issue_drive_cmd(0.0, self.spin_speed)
                else:
                    msg.data = "Here 1"
                    # If the rover is back at its start heading (360 degrees), move to hex search
                    if abs(self.wrap(self.spin_start_heading - self.spin_target_angle, 0)) < 0.01:
                        self.drive_controller.issue_drive_cmd(0, self.spin_speed)
                        self.state = State.START_HEX_SEARCH
                    
                    # If the rover has spun self.aruco_spin_step_size, stop the rover and look for tag
                    if self.wrap(self.curr_heading - self.spin_target_angle, 0) > 0:
                        msg.data = "Here 2"
                        self.spin_stop = True
                        self.spin_stop_time = time.time()
                        self.drive_controller.issue_drive_cmd(0, 0)
                        self.drive_controller.stop()
                if self.correct_aruco_tag_found:
                    self.state = State.ARUCO_NAVIGATE
                elif self.correct_obj_found:
                    self.state = State.OBJECT_NAVIGATE
                self.debug_pub.publish(msg) # For debugging TODO: Remove

            elif self.state == State.START_HEX_SEARCH:
                self.nav_state.navigation_state = NavState.AUTONOMOUS_STATE
                self.hex_search_point = GPSTools.heading_distance_to_lat_lon(
                    self.hex_center_point,
                    self.hex_search_point_num * self.hex_seach_angle_difference,
                    self.hex_search_radius * self.hex_radius_factor
                )
                self.drive_controller.issue_path_cmd(self.hex_search_point.lat, self.hex_search_point.lon)
                self.state = State.HEX_SEARCH

            elif self.state == State.HEX_SEARCH:
                self.nav_state.navigation_state = NavState.AUTONOMOUS_STATE
                if GPSTools.distance_between_lat_lon(self.current_point, self.hex_search_point) < self.dist_tolerance:
                    self.state = State.START_SPIN_SEARCH
                    self.hex_search_point_num += 1
                    if (self.tag_id in [TagID.MALLET, TagID.BOTTLE]) and self.hex_search_point_num % 6 == 0:
                        self.hex_radius_factor += 1/3
                if self.correct_aruco_tag_found:
                    self.state = State.ARUCO_NAVIGATE
                elif self.correct_obj_found:
                    self.state = State.OBJECT_NAVIGATE

            elif self.state == State.ARUCO_NAVIGATE:
                self.set_speed(self.aruco_speed)
                self.nav_state.navigation_state = NavState.AUTONOMOUS_STATE
                if self.correct_aruco_tag_found: #f we have seen the correct tag in the last second, navigate using angle and distance
                    self.drive_controller.issue_aruco_autopilot_cmd(self.aruco_tag_angle, self.aruco_tag_distance)
                else: #if we have not seen the correct tag in the last second, navigate to the last known position
                    self.drive_controller.issue_path_cmd(self.current_aruco_point.lat, self.current_aruco_point.lon)
                if self.aruco_tag_distance < self.aruco_dist_tolerance and self.tag_id.value < 4:
                    self.get_logger().info('Successfully navigated to the aruco tag!')
                    self.state = State.TASK_COMPLETE
                    self.drive_controller.stop()

            elif self.state == State.OBJECT_NAVIGATE:
                self.set_speed(self.object_speed)
                self.nav_state.navigation_state = NavState.AUTONOMOUS_STATE
                if self.correct_obj_found:
                    self.drive_controller.issue_aruco_autopilot_cmd(self.obj_angle, self.obj_distance)
                if self.obj_distance < self.obj_dist_tolerance:
                    self.get_logger().info('Successfully navigated to the object!')
                    self.state = State.TASK_COMPLETE
                    self.drive_controller.stop()

            elif self.state == State.TASK_COMPLETE:
                self.nav_state.navigation_state = NavState.ARRIVAL_STATE
                self.drive_controller.issue_drive_cmd(0, 0)
                self.drive_controller.stop()
                self.a_task_complete = True

                self.reset_state_variables()

                # Pop off the completed task
                if len(self.waypoints) > 0:
                    self.last_waypoint = self.waypoints.popleft()
                    self.get_logger().info('Completed waypoint popped off')

                self.state = State.MANUAL

            elif self.state == State.START_ABORT_STATE:
                self.nav_state.navigation_state = NavState.AUTONOMOUS_STATE
                self.set_speed(self.navigate_speed)
                self.drive_controller.issue_path_cmd(self.abort_lat, self.abort_lon)
                self.state = State.ABORT_STATE

            elif self.state == State.ABORT_STATE:
                self.nav_state.navigation_state = NavState.AUTONOMOUS_STATE
                if GPSTools.distance_between_lat_lon(self.current_point, self.abort_point) < self.abort_dist_tolerance:
                    self.get_logger().info("Reached abort point!")
                    self.state = State.MANUAL
                    self.drive_controller.issue_drive_cmd(0, 0)
                    self.drive_controller.stop()

            else:
                self.get_logger().warn("State machine state selected is not a real state")


        else:
            if self.state != State.MANUAL:
                self.drive_controller.issue_drive_cmd(0, 0)
                self.drive_controller.stop()
            self.state = State.MANUAL
            self.nav_state.navigation_state = NavState.TELEOPERATION_STATE

        self.nav_state_pub.publish(self.nav_state)
        self.publish_status()
        self.check_detections()

    def publish_status(self):
        msg = RoverState()
        msg.state = str(self.state.name.upper())
        self.rover_state_pub.publish(msg)


def main(args=None):
    """
    Main function for the state machine
    
    Continuous loop that publishes status and checks for updates from the GUI
    """
    rclpy.init(args=args)
    autonomy_state_machine = AutonomyStateMachine()
    rclpy.spin(autonomy_state_machine)

    # Clean up when shutting down
    autonomy_state_machine.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
