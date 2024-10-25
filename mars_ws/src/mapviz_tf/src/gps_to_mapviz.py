#!/usr/bin/env python3

import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

import numpy as np

from gps_common.msg import GPSFix
from rover_msgs.msg import RoverStateSingleton, AutonomyTaskInfo
from rover_msgs.srv import AutonomyWaypoint, AutonomyWaypointResponse
from ublox.msg import PositionVelocityTime

def spin_in_background():
    executor = rclpy.get_global_executor()
    try:
        executor.spin()
    except ExternalShutdownException:
        pass

ROS_RATE = 15


class GPSPublisher(Node):
    def __init__(self): #left old publishers to show conversion, will remove before final commit and push.
        # self.rover_filtered_gps_pub = rclpy.Publisher(
        #     "/GPSFix_rover_filtered", GPSFix, queue_size=1
        # )
        self.rover_filtered_gps_pub = self.create_publisher(GPSFix, "/GPSFix_rover_filtered", 1)
        # self.rover_unfiltered_gps_pub = rclpy.Publisher(
        #     "/GPSFix_rover_unfiltered", GPSFix, queue_size=1
        # )
        self.rover_unfiltered_gps_pub = self.create_publisher(GPSFix, "/GPSFix_rover_unfiltered", 1)
        self.base_gps_pub = self.create_publisher(GPSFix, "/GPSFix_base", queue_size=1)

        # Add waypoints to the plotter
        # self.gps_plotter_pub = rclpy.Publisher("/GPS_waypoint_plotter", GPSFix, queue_size=1)
        self.gps_plotter_pub = self.create_publisher(GPSFix, "/GPS_waypoint_plotter", 1)
        #self.gps_plotter_sub = rospy.Subscriber("/mapviz_GPS_waypoint", GPSFix, self.plotter_callback)

        self.gps_plotter_srv = rclpy.Service('/GPS_waypoint_plotter', AutonomyWaypoint, self.set_all_tasks_callback)   # TODO: Add this to the GUI buttons TODO: change this to rclpy


        self.rover_state_singleton_sub = rclpy.Subscriber(
            "/odometry/rover_state_singleton",
            RoverStateSingleton,
            self.singleton_callback,
        )

        self.base_sub = rclpy.Subscriber(
            "/base/PosVelTime", PositionVelocityTime, self.base_callback
        )

        self.rover_state_singleton = RoverStateSingleton()
        self.base_gps = PositionVelocityTime()

        print("Setting done for GPS")

    def singleton_callback(self, msg):
        self.rover_state_singleton = msg

    def base_callback(self, msg):
        self.base_gps = msg

    def publish_gps_data(self):
        yaw = self.rover_state_singleton.map_yaw
        self._publish_base_gps_data(self.base_gps_pub, self.base_gps)
        self._publish_rover_gps_data(
            self.rover_filtered_gps_pub, self.rover_state_singleton.filter_gps, yaw
        )
        self._publish_rover_gps_data(
            self.rover_unfiltered_gps_pub, self.rover_state_singleton.gps, yaw
        )

    def _publish_rover_gps_data(self, gps_publisher, gps, yaw):
        GPSFix_msg = GPSFix(latitude=gps.latitude, longitude=gps.longitude, track=yaw)
        gps_publisher.publish(GPSFix_msg)

    def _publish_base_gps_data(self, gps_publisher, posVelTime):
        GPSFix_msg = GPSFix(
            latitude=posVelTime.lla[0], longitude=posVelTime.lla[1], track=0
        )
        gps_publisher.publish(GPSFix_msg)

    def set_all_tasks_callback(self, task_list:AutonomyWaypoint) -> AutonomyWaypointResponse:
        '''
        Puts all of the waypoints from the GUI into a queue so the rover can autonomously do the whole mission.
        '''
        print('in set_all_tasks_callback')
        tasks: list[AutonomyTaskInfo] = task_list.task_list
        for task_info in tasks:
            gps_msg = GPSFix(latitude=task_info.latitude,longitude=task_info.longitude)
            self.gps_plotter_pub.publish(gps_msg)
        print('Exiting set_all_tasks_callback')

        response = AutonomyWaypointResponse()
        response.success = True
        response.message = 'Adding waypoints was successful'
        return response
        



if __name__ == "__main__":
    rclpy.init()
    t = threading.Thread(target=spin_in_background)
    t.start()
    node = rclpy.create_node("gps_to_mapviz")
    rclpy.get_global_executor().add_node(node)
    rate = node.create_rate(ROS_RATE)   #rospy.Rate(ROS_RATE)
    gps_pub = GPSPublisher()
    while rclpy.ok():
        gps_pub.publish_gps_data()
        # rate.sleep()    - Theoretically t.start and join do this.
    t.join()