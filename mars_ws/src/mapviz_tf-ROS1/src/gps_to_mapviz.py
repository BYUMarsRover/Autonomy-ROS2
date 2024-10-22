#!/usr/bin/env python3

import rospy

import numpy as np

from gps_common.msg import GPSFix
from rover_msgs.msg import RoverStateSingleton, AutonomyTaskInfo
from rover_msgs.srv import AutonomyWaypoint, AutonomyWaypointResponse
from ublox.msg import PositionVelocityTime


ROS_RATE = 15


class GPSPublisher:
    def __init__(self):
        self.rover_filtered_gps_pub = rospy.Publisher(
            "/GPSFix_rover_filtered", GPSFix, queue_size=1
        )
        self.rover_unfiltered_gps_pub = rospy.Publisher(
            "/GPSFix_rover_unfiltered", GPSFix, queue_size=1
        )
        self.base_gps_pub = rospy.Publisher("/GPSFix_base", GPSFix, queue_size=1)

        # Add waypoints to the plotter
        self.gps_plotter_pub = rospy.Publisher("/GPS_waypoint_plotter", GPSFix, queue_size=1)
        #self.gps_plotter_sub = rospy.Subscriber("/mapviz_GPS_waypoint", GPSFix, self.plotter_callback)

        self.gps_plotter_srv = rospy.Service('/GPS_waypoint_plotter', AutonomyWaypoint, self.set_all_tasks_callback)   # TODO: Add this to the GUI buttons


        self.rover_state_singleton_sub = rospy.Subscriber(
            "/odometry/rover_state_singleton",
            RoverStateSingleton,
            self.singleton_callback,
        )

        self.base_sub = rospy.Subscriber(
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
    rospy.init_node("gps_to_mapviz")
    rate = rospy.Rate(ROS_RATE)
    gps_pub = GPSPublisher()
    while not rospy.is_shutdown():
        gps_pub.publish_gps_data()
        rate.sleep()
