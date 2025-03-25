#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import numpy as np
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from mapviz_tf.lat_lon_meter_convertor import LatLonConvertor
from rover_msgs.msg import RoverStateSingleton
import utm
import yaml
import os

class RoverTransformBroadcaster(Node):
    def __init__(self):
        super().__init__('rover_tf_broadcaster')
        
        # Find origin
        self.declare_parameter('MAPVIZ_LOCATION', 'hanksville') # Declare Mapviz Location Parameter (passed in from launch file)
        mapviz_location = self.get_parameter('MAPVIZ_LOCATION').value # Extract location
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

        # Initialize variable to store rover position in x/y format
        self.rover_position = None

        # Initialize TransformBroadcaster
        self.tf_broadcaster = TransformBroadcaster(self)


        # Subscribers
        self.odom_sub = self.create_subscription(RoverStateSingleton, "/odometry/rover_state_singleton", self.odom_callback, 10)

    def odom_callback(self, msg):

        # Create TransformStamped message
        t = TransformStamped()

        # Header information
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "rover"

        # Convert latitude and longitude to x, y positions in map frame for use in the transform from rover to map
        self.rover_position = np.array((utm.from_latlon(msg.gps.latitude, msg.gps.longitude)[0:2])[::-1])

        # Subtract the origin off
        self.rover_position = self.rover_position - self.origin

        t.transform.translation.x = self.rover_position[1]
        t.transform.translation.y = self.rover_position[0]
        t.transform.translation.z = 0.0

        # map_yaw only quaternion TODO: change this to subscribe to the imu message that has the pose
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = np.sin(np.deg2rad(-msg.map_yaw)/2.0) # NOTE: negative because NED frame
        t.transform.rotation.w = np.cos(np.deg2rad(-msg.map_yaw)/2.0)

        # Orientation (rotation from Odometry message)
        # t.transform.rotation = msg.pose.pose.orientation

        # Send transform
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = RoverTransformBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
