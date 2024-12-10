#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from mapviz_tf.lat_lon_meter_convertor import LatLonConvertor


class RoverTransformBroadcaster(Node):
    def __init__(self):
        super().__init__('rover_tf_broadcaster')
        
        # Initialize variables
        self.origin = {}
        self.rover_position = {"x": np.nan, "y": np.nan}

        # Initialize TransformBroadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribers
        self.lla_sub = self.create_subscription(NavSatFix, "/ins/lla", self.lla_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, "/odometry/filtered", self.odom_callback, 10)

        # Lat/Lon Converter
        self.latlonconv = LatLonConvertor()

    def odom_callback(self, msg):
        if np.isnan(self.rover_position["x"]) or np.isnan(self.rover_position["y"]):
            self.get_logger().info("Rover Position Never Updated...")
            return

        # Create TransformStamped message
        t = TransformStamped()

        # Header information
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "rover"

        # Position (translation)
        t.transform.translation.x = self.rover_position["x"]
        t.transform.translation.y = self.rover_position["y"]
        t.transform.translation.z = 0.0

        # Orientation (rotation from Odometry message)
        t.transform.rotation = msg.pose.pose.orientation

        # Send transform
        self.tf_broadcaster.sendTransform(t)

    def lla_callback(self, msg):
        # Convert latitude and longitude to x, y positions
        self.rover_position = self.latlonconv.convert_to_meters(msg.latitude, msg.longitude)

def main(args=None):
    rclpy.init(args=args)
    node = RoverTransformBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
