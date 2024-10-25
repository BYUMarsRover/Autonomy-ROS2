#!/usr/bin/env python3

import threading
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

import numpy as np
import tf
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from lat_lon_meter_convertor import LatLonConvertor

def spin_in_background():
    executor = rclpy.get_global_executor()
    try:
        executor.spin()
    except ExternalShutdownException:
        pass

class RoverTransformBroadcaster(Node):
    def __init__(self):
        self.origin = {}
        self.rover_position = {"x": np.nan, "y": np.nan}

        self.lla_sub = self.create_subscriber(NavSatFix, "/ins/lla", self.lla_callback)
        self.odom_sub = self.create_subscriber(Odometry, "/odometry/filtered", self.odom_callback)

        self.latlonconv = LatLonConvertor()

    def odom_callback(self, msg):
        br = tf.TransformBroadcaster()
        if np.isnan(self.rover_position["x"]) or np.isnan(self.rover_position["y"]):
            print("Rover Position Never Updated...")
            return
        translation = (self.rover_position["x"], self.rover_position["y"], 0)
        rotation = []
        rotation.append(msg.pose.pose.orientation.x)
        rotation.append(msg.pose.pose.orientation.y)
        rotation.append(msg.pose.pose.orientation.z)
        rotation.append(msg.pose.pose.orientation.w)
        br.sendTransform(translation, rotation, rclpy.Time.now(), "rover", "map")

    def lla_callback(self, msg):
        self.rover_position = self.latlonconv.convert_to_meters(
            msg.latitude, msg.longitude
        )


if __name__ == "__main__":
    rclpy.init()
    node = rclpy.create_node("rover_tf_broadcaster")
    # rospy.init_node("rover_tf_broadcaster")
    transform = RoverTransformBroadcaster()
    rclpy.spin()
