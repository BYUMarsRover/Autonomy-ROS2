#!/usr/bin/env python3

#TODO: we need to move over the params, and we need to make sure that the service callbacks are right

import rclpy
from rclpy.node import Node
from rover_msgs.msg import AutonomyTaskInfo, RoverStateSingleton, RoverState, NavStatus, FiducialData, FiducialTransformArray
from rover_msgs.srv import SetFloat32, AutonomyAbort, AutonomyWaypoint  # TODO: Need to call the service in the GUI somewhere.
from std_srvs.srv import SetBool
# from mobility.src.drive_controller_api import DriveControllerAPI
from autonomy.drive_controller_api import DriveControllerAPI
from autonomy.GPSTools import GPSTools, GPSCoordinate
from enum import Enum
import numpy as np
import time
# from ament_index_pyuthon.packages import get_package_share_directory
# import os

class State(Enum):
    MANUAL = 1
    SEARCH_FOR_WRONG_TAG = 2
    START_POINT_NAVIGATION = 3
    POINT_NAVIGATION = 4
    START_ARUCO_SPIN_SEARCH = 5
    ARUCO_SPIN_SEARCH = 6
    START_ARUCO_HEX_SEARCH = 7
    ARUCO_HEX_SEARCH = 8
    ARUCO_NAVIGATE = 9
    # START_ARUCO_GATE_NAVIGATION = 10
    # ARUCO_GATE_NAVIGATION = 11
    # ARUCO_GATE_ORIENTATION = 12
    # ARUCO_GATE_APPROACH = 13
    # ARUCO_GATE_PAST = 14
    TASK_COMPLETE = 15
    START_ABORT_STATE = 16
    ABORT_STATE = 17

class TagID(Enum):
    AR_TAG_1 = 1
    AR_TAG_2 = 2
    AR_TAG_3 = 3
    BOTTLE = 4
    MALLET = 5
    GPS_ONLY = 6
    
class AutonomyStateMachine(Node):
    def __init__(self):
        super().__init__('state_machine')
        print('in init AutonomyStateMachine')
        self.get_logger().info('Autonomy state machine node initialized')

        # Subscribers
        # self.task_subs = rospy.Subscriber(
        #     '/autonomy/autonomy_task_info', AutonomyTaskInfo, self.set_task_callback, queue_size=1)
        self.rover_state_singleton_sub = self.create_subscription(RoverStateSingleton,'/odometry/rover_state_singleton', self.rover_state_singleton_callback, 10)
        self.sub_ar_detection_logi = self.create_subscription(FiducialTransformArray, '/aruco_detect_logi/fiducial_transforms', self.ar_tag_callback, 10)
        self.sub_ar_detection_zed = self.create_subscription(FiducialTransformArray, '/aruco_detect_zed/fiducial_transforms', self.ar_tag_callback, 10)

        # Publishers
        self.aruco_pose_pub = self.create_publisher(FiducialData, "/autonomy/aruco_pose", 1)

        self.nav_state_pub = self.create_publisher(RoverState, "/rover_status", 1)
        self.status_pub = self.create_publisher(NavStatus, '/nav_status', 1)


        # Services
        self.srv_switch_auto = self.create_service(SetBool,'/autonomy/enable_autonomy', self.enable)
        self.srv_switch_abort = self.create_service(AutonomyAbort, '/autonomy/abort_autonomy', self.abort)
        self.task_srvs = self.create_service(AutonomyWaypoint, '/AU_waypoint_service', self.set_all_tasks_callback)   # TODO: Add this to the GUI buttons

        #Declare Parameters
        self.declare_parameter('distance_tolerance', 1.0)
        self.declare_parameter('aruco_distance_tolerance', 5.0)
        self.declare_parameter('abort_distance_tolerance', 2.0)
        self.declare_parameter('hex_search_radius', 17.0)
        self.declare_parameter('navigate_speed', 1.0)
        self.declare_parameter('aruco_speed', 0.3)
        self.declare_parameter('aruco_spin_speed', 30.0)
        # self.declare_parameter('aruco_gate_spin_speed')
        self.declare_parameter('aruco_alpha_lpf', 0.5)
        self.declare_parameter('aruco_spin_step_size', 0.6981)
        self.declare_parameter('aruco_spin_delay_time', 1.2)
        self.declare_parameter('wrong_aruco_backup_distance', 7.0)
        # self.declare_parameter('aruco_gate_approach_distance', 6.0)
        self.declare_parameter('hex_seach_angle_difference', 50.0)

        #Get Parameters
        self.distance_tolerance = self.get_parameter('distance_tolerance').get_parameter_value().double_value
        self.abort_dist_tolerance = self.get_parameter('abort_distance_tolerance').get_parameter_value().double_value
        self.aruco_dist_tolerance = self.get_parameter('aruco_distance_tolerance').get_parameter_value().double_value
        self.hex_search_radius = self.get_parameter('hex_search_radius').get_parameter_value().double_value
        self.navigate_speed = self.get_parameter('navigate_speed').get_parameter_value().double_value
        self.aruco_speed = self.get_parameter('aruco_speed').get_parameter_value().double_value
        self.aruco_spin_speed = self.get_parameter('aruco_spin_speed').get_parameter_value().double_value
        # self.aruco_gate_spin_speed = rospy.get_param('aruco_gate_spin_speed')
        self.aruco_alpha_lpf = self.get_parameter('aruco_alpha_lpf').get_parameter_value().double_value
        self.aruco_spin_step_size = self.get_parameter('aruco_spin_step_size').get_parameter_value().double_value
        self.aruco_spin_delay_time = self.get_parameter('aruco_spin_delay_time').get_parameter_value().double_value
        self.wrong_aruco_backup_distance = self.get_parameter('wrong_aruco_backup_distance').get_parameter_value().double_value
        # self.aruco_gate_approach_distance = rospy.get_param('aruco_gate_approach_distance')
        self.hex_seach_angle_difference = self.get_parameter('hex_seach_angle_difference').get_parameter_value().double_value

        #self.rover_nav_state = RoverState() #thats a message?

        self.drive_controller = DriveControllerAPI(self)

        # self.state = State.MANUAL

        # self.enabled = False

        # self.correct_aruco_tag_found = False
        # # self.both_aruco_tags_found = False

        # self.aruco_tag_distance = None
        # self.aruco_tag_angle = None

        # self.post_a_distance = None
        # self.post_a_angle = None
        # self.post_b_distance = None
        # self.post_b_angle = None
        # self.ar_callback_see_time = 0

        # # TODO: there needs to be a better way to deal with this AttributeError exception
        # # there should be a try catch that detects if one of these variables are missing
        # # perhaps it can return something to the gui to display on the screen? Should
        # # we make the state_loop callback a service that can return data?

        # self.target_latitude = 0
        # self.target_longitude = 0
        # self.target_elvevation = 0
        # self.target_point = GPSCoordinate(self.target_latitude, self.target_longitude, 0)
        # self.curr_latitude = 0
        # self.curr_longitude = 0
        # self.curr_elvevation = 0
        # self.current_point = GPSCoordinate(self.curr_latitude, self.curr_longitude, self.curr_elvevation)  
        # self.tag_id = TagID.GPS_ONLY

        # self.correct_aruco_tag_found = False
        # self.wrong_aruco_tag_found = False
        # # self.both_aruco_tags_found = False
        
        # self.i = 0

        # # Data structure to hold all of the waypoints at a time
        # self.waypoints: deque[AutonomyTaskInfo] = deque()
        # self.last_waypoint: AutonomyTaskInfo = None
        # self.is_continue_to_next_waypoint = False
        # self.is_start_timer = True

    def set_task_callback(self, task_info:AutonomyTaskInfo):
        print('in set_task_callback')
        self.target_latitude = task_info.latitude
        self.target_longitude = task_info.longitude
        # Set tag ID
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
        self.target_point = GPSCoordinate(self.target_latitude, self.target_longitude, 0)

    def set_all_tasks_callback(self, request: AutonomyWaypoint.Request, response: AutonomyWaypoint.Response) -> AutonomyWaypoint.Response:
        '''
        Puts all of the waypoints from the GUI into a queue so the rover can autonomously do the whole mission.
        '''
        print('in set_all_tasks_callback')
        tasks = request.task_list  # Access the list of AutonomyTaskInfo
        for task in tasks:
            self.waypoints.append()  # Append each task to the waypoints NOT GONNA APPEND NUTHING SO IT DON'T BREAK
        print('Waypoints: ', self.waypoints)
        print('Exiting set_all_tasks_callback')

        response.success = True
        response.message = 'Adding waypoints was successful'
        return response

    def set_current_task(self):
        '''
        Sets current task to be the next one in the deque. Note that this also assumes that the TASK_COMPLETE task has popped
        the first waypoint off the front of the deque.
        TODO: Confirm that popping off the completed task is most clearly done in the TASK_COMPLETE state, or if it is better
        to pass in a task_complete parameter that tells this function to pop or not pop
        '''
        current_task: AutonomyTaskInfo = self.waypoints[0]
        self.set_task_callback(current_task)

        # TODO: Do I want to make this be the current location or the first waypoint?
        if self.last_waypoint == None:
            self.last_waypoint = current_task

    def rover_state_singleton_callback(self, msg:RoverStateSingleton):
        self.curr_latitude = msg.gps.latitude
        self.curr_longitude = msg.gps.longitude
        self.curr_elvevation = msg.gps.altitude
        self.current_point = GPSCoordinate(self.curr_latitude, self.curr_longitude, self.curr_elvevation)
        self.curr_heading = np.deg2rad(msg.map_yaw)

    def start_timer(self):
        if self.is_start_timer:
            # Create a one-shot timer (10 seconds)
            self.timer = self.create_timer(10.0, self.wait_timer_callback)
            self.is_start_timer = False

    def wait_timer_callback(self):
        # Timer callback logic
        self.is_continue_to_next_waypoint = True
        self.get_logger().info('Timer callback triggered, continuing to the next waypoint')
        self.timer.cancel()

    def ar_tag_callback(self, msg:FiducialTransformArray):
        # print("in ar_tag_callback")
        if(len(msg.transforms) == 1):
            # print("found 1 tag")
            if(self.aruco_tag_distance is None):
                self.aruco_tag_distance = np.sqrt(msg.transforms[0].transform.translation.x**2 + msg.transforms[0].transform.translation.z**2)
                self.aruco_tag_angle = - np.arctan(msg.transforms[0].transform.translation.x/msg.transforms[0].transform.translation.z)
            else:
                self.aruco_tag_distance = self.aruco_tag_distance * self.aruco_alpha_lpf + np.sqrt(msg.transforms[0].transform.translation.x**2 + msg.transforms[0].transform.translation.z**2) * (1 - self.aruco_alpha_lpf)
                self.aruco_tag_angle = self.aruco_tag_angle * self.aruco_alpha_lpf - np.arctan(msg.transforms[0].transform.translation.x/msg.transforms[0].transform.translation.z) * (1 - self.aruco_alpha_lpf)
            self.current_aruco_point = GPSTools.heading_distance_to_lat_lon(self.current_point, -np.rad2deg(self.curr_heading + self.aruco_tag_angle), self.aruco_tag_distance)
            # print("tag is {}m away at an angle of {} degrees".format(self.aruco_tag_distance, self.aruco_tag_angle))
            # print("tag at {}".format(np.rad2deg(self.curr_heading + self.aruco_tag_angle)))
            self.aruco_pose = FiducialData()
            self.aruco_pose.angle_offset = self.aruco_tag_angle
            self.aruco_pose.dist_to_fiducial = self.aruco_tag_distance
            self.aruco_pose_pub.publish(self.aruco_pose)

            self.ar_callback_see_time =  time.time()

            if(msg.transforms[0].fiducial_id == self.tag_id.value): # Gate traversal? or (msg.transforms[0].fiducial_id == 5 and self.tag_id == 4)):
                print("is correct tag: ", str(self.tag_id.value))
                self.correct_aruco_tag_found = True
                self.wrong_aruco_tag_found = False
            else:
                print("is not correct tag. tagID: " + str(msg.transforms[0].fiducial_id) + " Correct id: " + str(self.tag_id.value))
                self.correct_aruco_tag_found = False
                self.wrong_aruco_tag_found = True
            # self.both_aruco_tags_found = False
        # GATE TRAVERSAL TODO: Check that this doesn't cause any issues!
        # elif(len(msg.transforms) == 2):
        #     print("found 2 tags")

        #     self.ar_callback_see_time =  time.time()
            
        #     if(self.tag_id == 4 and msg.transforms[0].fiducial_id == 4 and msg.transforms[1].fiducial_id == 5):
        #         self.both_aruco_tags_found = True
        #         self.correct_aruco_tag_found = True
        #         self.wrong_aruco_tag_found = False
        #         if(self.post_a_distance is None):
        #             self.post_a_distance = np.sqrt(msg.transforms[0].transform.translation.x**2 + msg.transforms[0].transform.translation.z**2)
        #             self.post_a_angle = - np.arctan(msg.transforms[0].transform.translation.x/msg.transforms[0].transform.translation.z)
        #             self.post_b_distance = np.sqrt(msg.transforms[1].transform.translation.x**2 + msg.transforms[1].transform.translation.z**2)
        #             self.post_b_angle = - np.arctan(msg.transforms[1].transform.translation.x/msg.transforms[1].transform.translation.z)
        #         else:
        #             self.post_a_distance = self.post_a_distance * self.aruco_alpha_lpf + np.sqrt(msg.transforms[0].transform.translation.x**2 + msg.transforms[0].transform.translation.z**2) * (1 - self.aruco_alpha_lpf)
        #             self.post_a_angle = self.post_a_angle * self.aruco_alpha_lpf - np.arctan(msg.transforms[0].transform.translation.x/msg.transforms[0].transform.translation.z) * (1 - self.aruco_alpha_lpf)
        #             self.post_b_distance = self.post_b_distance * self.aruco_alpha_lpf + np.sqrt(msg.transforms[1].transform.translation.x**2 + msg.transforms[1].transform.translation.z**2) * (1 - self.aruco_alpha_lpf)
        #             self.post_b_angle = self.post_b_angle * self.aruco_alpha_lpf - np.arctan(msg.transforms[1].transform.translation.x/msg.transforms[1].transform.translation.z) * (1 - self.aruco_alpha_lpf)
        #         self.post_a_point = GPSTools.heading_distance_to_lat_lon(self.current_point, -np.rad2deg(self.curr_heading + self.post_a_angle), self.post_a_distance)
        #         self.post_b_point = GPSTools.heading_distance_to_lat_lon(self.current_point, -np.rad2deg(self.curr_heading + self.post_b_angle), self.post_b_distance)
                
        #         self.aruco_pose = FiducialData()
        #         self.aruco_pose.angle_offset = (self.post_a_angle + self.post_b_angle)/2
        #         self.aruco_pose.dist_to_fiducial = (self.post_a_distance + self.post_b_distance)/2
        #         self.aruco_pose_pub.publish(self.aruco_pose)
        #     elif(self.tag_id == 4 and msg.transforms[0].fiducial_id == 5 and msg.transforms[1].fiducial_id == 4):
        #         self.both_aruco_tags_found = True
        #         self.correct_aruco_tag_found = True
        #         self.wrong_aruco_tag_found = False
        #         if(self.post_a_distance is None):
        #             self.post_a_distance = np.sqrt(msg.transforms[1].transform.translation.x**2 + msg.transforms[1].transform.translation.z**2)
        #             self.post_a_angle = - np.arctan(msg.transforms[1].transform.translation.x/msg.transforms[1].transform.translation.z)
        #             self.post_b_distance = np.sqrt(msg.transforms[0].transform.translation.x**2 + msg.transforms[0].transform.translation.z**2)
        #             self.post_b_angle = - np.arctan(msg.transforms[0].transform.translation.x/msg.transforms[0].transform.translation.z)
        #         else:
        #             self.post_a_distance = self.post_a_distance * self.aruco_alpha_lpf + np.sqrt(msg.transforms[1].transform.translation.x**2 + msg.transforms[1].transform.translation.z**2) * (1 - self.aruco_alpha_lpf)
        #             self.post_a_angle = self.post_a_angle * self.aruco_alpha_lpf - np.arctan(msg.transforms[1].transform.translation.x/msg.transforms[1].transform.translation.z) * (1 - self.aruco_alpha_lpf)
        #             self.post_b_distance = self.post_b_distance * self.aruco_alpha_lpf + np.sqrt(msg.transforms[0].transform.translation.x**2 + msg.transforms[0].transform.translation.z**2) * (1 - self.aruco_alpha_lpf)
        #             self.post_b_angle = self.post_b_angle * self.aruco_alpha_lpf - np.arctan(msg.transforms[0].transform.translation.x/msg.transforms[0].transform.translation.z) * (1 - self.aruco_alpha_lpf)
        #         self.post_a_point = GPSTools.heading_distance_to_lat_lon(self.current_point, -np.rad2deg(self.curr_heading + self.post_a_angle), self.post_a_distance)
        #         self.post_b_point = GPSTools.heading_distance_to_lat_lon(self.current_point, -np.rad2deg(self.curr_heading + self.post_b_angle), self.post_b_distance)
                
        #         self.aruco_pose = FiducialData()
        #         self.aruco_pose.angle_offset = self.post_a_angle
        #         self.aruco_pose.dist_to_fiducial = self.post_a_distance
        #         self.aruco_pose_pub.publish(self.aruco_pose)
        #     else:
        #         self.correct_aruco_tag_found = False
        #         self.wrong_aruco_tag_found = True
        elif(time.time() - self.ar_callback_see_time > 1):
            self.correct_aruco_tag_found = False
            self.wrong_aruco_tag_found = False
            # self.both_aruco_tags_found = False
    
    def enable(self, request:SetBool.Request, response:SetBool.Response):
        print('in enable')
        self.enabled = request.data  # Access request data

        # Set the first task
        self.set_current_task()

        if self.enabled:
            print("Autonomy state machine is enabled!")
            self.state = State.SEARCH_FOR_WRONG_TAG
            # self.state = State.TASK_COMPLETE
            # self.state = State.START_ARUCO_SEARCH  # TODO: Change back to State.START_POINT_NAVIGATE
        else:
            print("Autonomy state machine is disabled!")

        # Set the response
        response.success = True
        response.message = "Autonomy state machine is {}!".format(self.enabled)

        return response
    
    def abort(self, request:AutonomyAbort.Request, response:AutonomyAbort.Response):
        print('in abort')
        self.abort_status = request.abort_status
        self.abort_lat = request.lat
        self.abort_lon = request.lon

        if self.abort_status:
            print("Aborting...")
            self.state = State.START_ABORT_STATE
        else:
            print("Cancelling abort!")
            self.state = State.MANUAL

        response.error = False
        response.message = "Abort is {}!".format(self.abort_status)

        return response
    
    def wrap(self, chi_1, chi_2):
        while chi_1 - chi_2 > np.pi:
            chi_1 = chi_1 - 2.0 * np.pi
        while chi_1 - chi_2 < -np.pi:
            chi_1 = chi_1 + 2.0 * np.pi
        return chi_1
    
    def set_autopilot_speed(self, speed):
        print("Setting autopilot speed...")

        # Create a service client for the speed factor service
        self.srv_autopilot_speed = self.create_client(SetFloat32, '/mobility/speed_factor')
        
        # Wait until the service is available
        if not self.srv_autopilot_speed.wait_for_service(timeout_sec=3.0): #Don't know what would be the appropriate time to wait here
            self.get_logger().error("Service /mobility/speed_factor not available!")
            return False
        print("Service is live")

        # Create a request object
        self.autopilot_speed_request = SetFloat32.Request()  # Create a request instance
        self.autopilot_speed_request.data = speed
        print("Executing service...")
        
        # Send the request and wait for the response
        future = self.srv_autopilot_speed.call_async(self.autopilot_speed_request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            print("Service executed!")
            return future.result().success
        else:
            self.get_logger().error("Service call failed!")
            return False


    def stateLoop(self):
        # rospy.logdebug('In State Loop')
        
        self.i += 1
        if self.i % 10==0:
            disp = True
        else:
            disp = False
        if disp:
            # print("enabled? == {}".format(self.enabled))
            # print("state? == {}".format(self.state))
            pass
        if(self.enabled):
            
            if(self.state == State.MANUAL):
                if disp: print('state is manual')
                self.rover_nav_state.navigation_state = RoverState.TELEOPERATION_STATE
            elif(self.state == State.SEARCH_FOR_WRONG_TAG):
                self.rover_nav_state.navigation_state = RoverState.AUTONOMOUS_STATE
                if(self.wrong_aruco_tag_found and self.aruco_tag_distance < self.wrong_aruco_backup_distance):
                    self.drive_controller.issue_drive_cmd(-2,0)
                    print("going backwards")
                else:
                    self.drive_controller.issue_drive_cmd(0,0)
                    self.state = State.START_POINT_NAVIGATION
            elif(self.state == State.START_POINT_NAVIGATION):
                if disp: print('state is point_nav')
                self.rover_nav_state.navigation_state = RoverState.AUTONOMOUS_STATE
                self.set_autopilot_speed(self.navigate_speed)
                # print("target latitude: ", self.target_latitude)
                # print("target longitude: ", self.target_longitude)
                self.drive_controller.issue_path_cmd(self.target_latitude, self.target_longitude)
                self.state = State.POINT_NAVIGATION
            elif(self.state == State.POINT_NAVIGATION):
                self.rover_nav_state.navigation_state = RoverState.AUTONOMOUS_STATE
                # self.set_autopilot_speed(self.navigate_speed)
                if(GPSTools.distance_between_lat_lon(self.current_point, self.target_point) < self.distance_tolerance):
                    if(self.tag_id == TagID.GPS_ONLY):
                        print('GPS Task is complete!')
                        self.state = State.TASK_COMPLETE
                        self.drive_controller.stop()
                    elif (self.tag_id == TagID.MALLET):
                        # TODO: Do something cool for the mallet code
                        print('Mallet task is complete! This is because we have no working Mallet code!')
                        self.state = State.TASK_COMPLETE
                        self.drive_controller.stop()
                    elif (self.tag_id == TagID.BOTTLE):
                        # TODO: Do something cooler for the bottle code.
                        print('Bottle task is complete! This is because we have no working Bottle code!')
                        self.state = State.TASK_COMPLETE
                        self.drive_controller.stop()
                    else:
                        self.state = State.START_ARUCO_SPIN_SEARCH
                        self.hex_search_point_num = 0
                        self.hex_center_point = self.current_point
                if(self.correct_aruco_tag_found):
                    self.state = State.ARUCO_NAVIGATE
            elif(self.state == State.START_ARUCO_SPIN_SEARCH):
                self.rover_nav_state.navigation_state = RoverState.AUTONOMOUS_STATE
                self.aruco_spin_start_heading = self.wrap(self.curr_heading,0)   #in radians
                # self.aruco_half_spin_heading = self.wrap(self.curr_heading - np.pi,0) #in radians
                self.aruco_spin_stop = False
                self.aruco_spin_target_angle = self.wrap(self.curr_heading + self.aruco_spin_step_size,0)
                print("target: {}".format(self.aruco_spin_target_angle))
                # self.set_autopilot_speed(self.aruco_speed)
                self.drive_controller.issue_drive_cmd(0,self.aruco_spin_speed)
                self.state = State.ARUCO_SPIN_SEARCH

            elif(self.state == State.ARUCO_SPIN_SEARCH):
                if disp: print('state is aruco_spin_search')
                self.rover_nav_state.navigation_state = RoverState.AUTONOMOUS_STATE
                if(self.aruco_spin_stop):
                    print("am stopped")
                    if(time.time() - self.aruco_spin_stop_time > self.aruco_spin_delay_time):
                        self.aruco_spin_stop = False
                        self.aruco_spin_target_angle = self.wrap(self.aruco_spin_target_angle + self.aruco_spin_step_size,0)
                        self.drive_controller.issue_drive_cmd(0,self.aruco_spin_speed)
                        print("spinning...")
                        print("target: {}".format(self.aruco_spin_target_angle))
                else:
                    print("am spinning")
                    print("target: {}".format(self.aruco_spin_target_angle))
                    if(abs(self.wrap(self.aruco_spin_start_heading - self.aruco_spin_target_angle,0)) < 0.01):
                        self.drive_controller.issue_drive_cmd(0,self.aruco_spin_speed)
                        self.state = State.START_ARUCO_HEX_SEARCH
                    if(self.wrap(self.curr_heading - self.aruco_spin_target_angle,0) > 0):
                        print("stopping...")
                        self.aruco_spin_stop = True
                        self.aruco_spin_stop_time = time.time()
                        self.drive_controller.issue_drive_cmd(0,0)
                        self.drive_controller.stop()
                if(self.correct_aruco_tag_found):
                    self.state = State.ARUCO_NAVIGATE

            elif(self.state == State.START_ARUCO_HEX_SEARCH):
                self.rover_nav_state.navigation_state = RoverState.AUTONOMOUS_STATE
                self.hex_search_point = GPSTools.heading_distance_to_lat_lon(self.hex_center_point, self.hex_search_point_num * self.hex_seach_angle_difference, self.hex_search_radius)
                self.drive_controller.issue_path_cmd(self.hex_search_point.lat, self.hex_search_point.lon)
                # self.set_autopilot_speed(self.navigate_speed)
                self.state = State.ARUCO_HEX_SEARCH

            elif(self.state == State.ARUCO_HEX_SEARCH):
                if disp: print('state is aruco hex search')
                self.rover_nav_state.navigation_state = RoverState.AUTONOMOUS_STATE
                if(GPSTools.distance_between_lat_lon(self.current_point, self.hex_search_point) < self.distance_tolerance):
                    self.state = State.START_ARUCO_SPIN_SEARCH
                    self.hex_search_point_num += 1
                if(self.correct_aruco_tag_found):
                    self.state = State.ARUCO_NAVIGATE
                       
            elif(self.state == State.ARUCO_NAVIGATE):
                if disp: print('state is aruco navigate')
                # if(self.correct_aruco_tag_found):
                self.set_autopilot_speed(self.aruco_speed)
                self.rover_nav_state.navigation_state = RoverState.AUTONOMOUS_STATE
                if(self.correct_aruco_tag_found):
                    self.drive_controller.issue_aruco_autopilot_cmd(self.aruco_tag_angle ,self.aruco_tag_distance)
                    # print("going at {} radians {} meters".format(self.aruco_tag_angle, self.aruco_tag_distance))
                else:
                    self.drive_controller.issue_path_cmd(self.current_aruco_point.lat, self.current_aruco_point.lon)
                    # print("going to point {}, {}".format(self.current_aruco_point.lat, self.current_aruco_point.lon))
                # if((self.aruco_tag_distance < self.aruco_gate_approach_distance and self.tag_id == 4) or self.both_aruco_tags_found):
                    # TODO: change state to something else
                    # self.state = State.START_ARUCO_GATE_NAVIGATION
                if(self.aruco_tag_distance < self.aruco_dist_tolerance and self.tag_id.value < 4):
                    if disp: print('Successfully navigated to the aruco tag!')
                    self.state = State.TASK_COMPLETE
                    self.drive_controller.stop()
                
            # elif(self.state == State.START_ARUCO_GATE_NAVIGATION):
            #     if disp: print('state is start_aruco_gate_navigation')
            #     self.rover_nav_state.navigation_state = RoverState.AUTONOMOUS_STATE
            #     if(self.both_aruco_tags_found):
            #         point_1 = GPSTools.heading_distance_to_lat_lon(GPSTools.midpoint_from_lat_lon(self.post_a_point, self.post_b_point), GPSTools.heading_between_lat_lon(self.post_a_point, self.post_b_point) - 90, self.aruco_gate_approach_distance)
            #         point_2 = GPSTools.heading_distance_to_lat_lon(GPSTools.midpoint_from_lat_lon(self.post_a_point, self.post_b_point), GPSTools.heading_between_lat_lon(self.post_a_point, self.post_b_point) + 90, self.aruco_gate_approach_distance)
            #         if(GPSTools.distance_between_lat_lon(self.current_point, point_1) < GPSTools.distance_between_lat_lon(self.current_point, point_2)):
            #             self.aruco_gate_start_point = point_1
            #             self.aruco_gate_end_point = point_2
            #             self.gate_heading_from_start = self.wrap(self.curr_heading*np.pi/180 + np.pi/2,0)
            #         else:
            #             self.aruco_gate_start_point = point_2
            #             self.aruco_gate_end_point = point_1
            #             self.gate_heading_from_start = self.wrap(self.curr_heading*np.pi/180 - np.pi/2,0)
            #     else:
            #         self.aruco_gate_start_point = GPSTools.heading_distance_to_lat_lon(self.current_aruco_point, np.rad2deg(self.curr_heading) - 90,0, self.aruco_gate_approach_distance)
            #         self.aruco_gate_end_point = GPSTools.heading_distance_to_lat_lon(self.current_aruco_point, np.rad2deg(self.curr_heading) + 90,0, self.aruco_gate_approach_distance)
            #         self.gate_heading_from_start = self.wrap(self.curr_heading*np.pi/180 + np.pi/2,0)
            #     self.set_autopilot_speed(self.navigate_speed)
            #     self.drive_controller.issue_path_cmd(self.aruco_gate_start_point.lat, self.aruco_gate_start_point.lon)
            #     self.state = State.ARUCO_GATE_NAVIGATION
            
            # elif(self.state == State.ARUCO_GATE_NAVIGATION):
            #     if disp: print('state is aruco_gate_navigation')
            #     self.rover_nav_state.navigation_state = RoverState.AUTONOMOUS_STATE
            #     if(GPSTools.distance_between_lat_lon(self.current_point, self.aruco_gate_start_point) < self.dist_tolerance):
            #         self.drive_controller.issue_drive_cmd(0,self.aruco_gate_spin_speed)
            #         self.state = State.ARUCO_GATE_ORIENTATION
            
            # elif(self.state == State.ARUCO_GATE_ORIENTATION):
            #     if disp: print('state is aruco_gate_orientation')
            #     self.rover_nav_state.navigation_state = RoverState.AUTONOMOUS_STATE
            #     if(abs(self.wrap(self.curr_heading - self.gate_heading_from_start,0)) < 5):
            #             self.state = State.ARUCO_GATE_APPROACH

            # elif(self.state == State.ARUCO_GATE_APPROACH):
            #     if disp: print('state is aruco_gate_approach')
            #     self.rover_nav_state.navigation_state = RoverState.AUTONOMOUS_STATE
            #     if(self.both_aruco_tags_found):
            #         angle = (self.post_a_angle + self.post_b_angle)/2
            #         distance = (self.post_a_distance + self.post_b_distance)/2
            #         self.drive_controller.issue_aruco_autopilot_cmd(angle, distance)
            #         if(abs(self.wrap(self.curr_heading - np.deg2rad(GPSTools.heading_between_lat_lon(self.post_a_point, self.post_b_point) - 90),0)) < abs(self.wrap(self.curr_heading - np.deg2rad(GPSTools.heading_between_lat_lon(self.post_a_point, self.post_b_point) + 90),0))):
            #             self.aruco_gate_end_point = GPSTools.heading_distance_to_lat_lon(GPSTools.midpoint_from_lat_lon(self.post_a_point, self.post_b_point), GPSTools.heading_between_lat_lon(self.post_a_point, self.post_b_point) - 90, self.aruco_gate_approach_distance)
            #         else:
            #             self.aruco_gate_end_point = GPSTools.heading_distance_to_lat_lon(GPSTools.midpoint_from_lat_lon(self.post_a_point, self.post_b_point), GPSTools.heading_between_lat_lon(self.post_a_point, self.post_b_point) + 90, self.aruco_gate_approach_distance)
            #     else:
            #         self.drive_controller.issue_drive_cmd(self.aruco_gate_end_point.lat, self.aruco_gate_end_point.lon)
            #         self.state = State.ARUCO_GATE_PAST

            # elif(self.state == State.ARUCO_GATE_PAST):
            #     if disp: print('state is aruco_gate_past')
            #     self.rover_nav_state.navigation_state = RoverState.AUTONOMOUS_STATE
            #     if(GPSTools.distance_between_lat_lon(self.current_point, self.aruco_gate_end_point) < self.dist_tolerance):
            #         self.state = State.TASK_COMPLETE
            #         self.drive_controller.issue_drive_cmd(0,0)
            #         self.drive_controller.stop()

            elif(self.state == State.TASK_COMPLETE):
                if disp: print('state is complete')
                self.rover_nav_state.navigation_state = RoverState.ARRIVAL_STATE
                self.drive_controller.issue_drive_cmd(0,0)
                self.drive_controller.stop()

                # Pop off the completed task
                if len(self.waypoints) > 0:
                    self.last_waypoint = self.waypoints.popleft()

                # Check to see if there is another task available and continue the mission. Also wait 10 seconds to signify to the judges.
                self.start_timer()
                if len(self.waypoints) > 0 and self.is_continue_to_next_waypoint:
                    print('Additional waypoints to complete!')
                    self.set_current_task()
                    
                    # Begin the new waypoint!
                    print('Beginning the new waypoint...')
                    self.state = State.SEARCH_FOR_WRONG_TAG

                    # Reset flag variables
                    self.is_continue_to_next_waypoint = False
                    self.is_start_timer = True

            elif(self.state == State.START_ABORT_STATE):
                if disp: print('state is start_abort_state')
                self.rover_nav_state.navigation_state = RoverState.AUTONOMOUS_STATE
                self.set_autopilot_speed(self.navigate_speed)
                print("abort latitude: ", self.abort_lat)
                print("abort longitude: ", self.abort_lon)
                self.drive_controller.issue_path_cmd(self.abort_lat, self.abort_lon)
                print("issued command")
                self.state = State.ABORT_STATE

            elif(self.state == State.ABORT_STATE):
                print("in abort state")
                self.rover_nav_state.navigation_state = RoverState.AUTONOMOUS_STATE
                # self.set_autopilot_speed(self.navigate_speed)
                if(GPSTools.distance_between_lat_lon(self.current_point, self.abort_point) < self.abort_dist_tolerance):
                    print("reached point!")
                    self.state = State.MANUAL
                    self.drive_controller.issue_drive_cmd(0,0)
                    self.drive_controller.stop()
            else:
                print("state selected is not a real state")
        else:
            if(self.state != State.MANUAL):
                # print("Stopping drive controller")
                self.drive_controller.issue_drive_cmd(0,0)
                self.drive_controller.stop()
                # print("I stopped!")
            self.state = State.MANUAL
            self.rover_nav_state.navigation_state = RoverState.TELEOPERATION_STATE
        self.nav_state_pub.publish(self.rover_nav_state)
        self.publishStatus()

        if self.i % 100 == 0:
            self.i = 0

    def publishStatus(self):
        """
        Publishes the rovers current status every state loop
        """
        msg = NavStatus()
        if self.state == State.MANUAL:
            msg.state = "MANUAL"
        elif self.state == State.SEARCH_FOR_WRONG_TAG:
            msg.state = "SEARCH_FOR_WRONG_TAG"
        elif self.state == State.START_POINT_NAVIGATION:
            msg.state = "START_POINT_NAVIGATION"
        elif self.state == State.POINT_NAVIGATION:
            msg.state = "POINT_NAVIGATION"
        elif self.state == State.START_ARUCO_SPIN_SEARCH:
            msg.state = "START_ARUCO_SPIN_SEARCH"
        elif self.state == State.ARUCO_SPIN_SEARCH:
            msg.state = "ARUCO_SPIN_SEARCH"
        elif self.state == State.START_ARUCO_HEX_SEARCH:
            msg.state = "START_ARUCO_HEX_SEARCH"
        elif self.state == State.ARUCO_HEX_SEARCH:
            msg.state = "ARUCO_HEX_SEARCH"
        elif self.state == State.ARUCO_NAVIGATE:
            msg.state = "ARUCO_NAVIGATE"
        # elif self.state == State.START_ARUCO_GATE_NAVIGATION:
        #     msg.state = "START_ARUCO_GATE_NAVIGATION"
        # elif self.state == State.ARUCO_GATE_NAVIGATION:
        #     msg.state = "ARUCO_GATE_NAVIGATION"
        # elif self.state == State.ARUCO_GATE_ORIENTATION:
        #     msg.state = "ARUCO_GATE_ORIENTATION"
        # elif self.state == State.ARUCO_GATE_APPROACH:
        #     msg.state = "ARUCO_GATE_APPROACH"
        # elif self.state == State.ARUCO_GATE_PAST:
        #     msg.state = "ARUCO_GATE_PAST"
        elif self.state == State.TASK_COMPLETE:
            msg.state = "TASK_COMPLETE"
        elif self.state == State.START_ABORT_STATE:
            msg.state = "START_ABORT_STATE"
        elif self.state == State.ABORT_STATE:
            msg.state = "ABORT_STATE"

        self.status_pub.publish(msg)

def main(args=None):
    """
    Main function for the state machine
    
    Continuous loop that publishes status and checks for updates from the GUI
    """
    rclpy.init(args=args)  # Initialize the ROS 2 Python client library

    # Create an instance of the AutonomyStateMachine node
    autonomy_state_machine = AutonomyStateMachine()

    rate = autonomy_state_machine.create_rate(10)  # ROS 2 rate (10 Hz)

    # Loop until ROS 2 is shut down
    while rclpy.ok():
        rclpy.spin_once(autonomy_state_machine)  # Process callbacks once per loop
        autonomy_state_machine.stateLoop()  # Call the state loop function
        rate.sleep()

    # Clean up when shutting down
    autonomy_state_machine.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()