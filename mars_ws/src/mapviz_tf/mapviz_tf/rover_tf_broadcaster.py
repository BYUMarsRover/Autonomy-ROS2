#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import numpy as np
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from visualization_msgs.msg import Marker
from rover_msgs.msg import IWCMotors
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
        self.IWC_sub = self.create_subscription(IWCMotors, '/IWC_motorControl', self.publish_direction_marker, 1)
        self.marker_pub = self.create_publisher(Marker, 'motor_control_marker', 10)  # Added marker publisher


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

    def calculate_direction(self, IWC_cmd_msg):
        """
        Calculates the intended direction of the robot based on the motor commands.

        Args:
            IWC_cmd_msg (IWCMotors): The motor command message.

        Returns:
            float: The intended direction in radians, relative to the robot's current orientation.
                   Returns None if there is an error.
        """
        # Extract wheel speeds and directions
        left_front_speed = IWC_cmd_msg.left_front_speed
        left_front_dir = IWC_cmd_msg.left_front_dir
        left_middle_speed = IWC_cmd_msg.left_middle_speed
        left_middle_dir = IWC_cmd_msg.left_middle_dir
        left_rear_speed = IWC_cmd_msg.left_rear_speed
        left_rear_dir = IWC_cmd_msg.left_rear_dir

        right_front_speed = IWC_cmd_msg.right_front_speed
        right_front_dir = IWC_cmd_msg.right_front_dir
        right_middle_speed = IWC_cmd_msg.right_middle_speed
        right_middle_dir = IWC_cmd_msg.right_middle_speed
        right_rear_speed = IWC_cmd_msg.right_rear_speed
        right_rear_dir = IWC_cmd_msg.right_rear_dir

        # Convert speeds and directions to signed velocities.  Forward is positive.
        left_front_velocity = left_front_speed if left_front_dir else -left_front_speed
        left_middle_velocity = left_middle_speed if left_middle_dir else -left_middle_speed
        left_rear_velocity = left_rear_speed if left_rear_dir else -left_rear_speed

        right_front_velocity = right_front_speed if right_front_dir else -right_front_speed
        right_middle_velocity = right_middle_speed if right_middle_dir else -right_middle_speed
        right_rear_velocity = right_rear_speed if right_rear_dir else -right_rear_speed

        # Simple model: Average left and right side velocities.
        left_velocity = (left_front_velocity + left_middle_velocity + left_rear_velocity) / 3.0
        right_velocity = (right_front_velocity + right_middle_velocity + right_rear_velocity) / 3.0

        # Handle the case where both sides have the same velocity (straight line)
        if left_velocity == right_velocity:
            return 0.0  # Straight ahead

        # Calculate the turning radius.  If left_velocity is zero, the robot turns about the left wheel, and vice-versa.
        if left_velocity == 0:
          return pi/2
        if right_velocity == 0:
          return -pi/2

        # Use the difference in velocities to determine the turning direction
        direction = atan2((right_velocity - left_velocity), (right_velocity + left_velocity))
        return direction

    def publish_direction_marker(self, IWC_cmd_msg):
        """Publishes a visualization_msgs/Marker arrow indicating the robot's intended direction."""
        marker = Marker()
        marker.header.frame_id = "rover"  # Or "odom" or whatever your robot's base frame is
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "Motor Control"
        marker.id = 0  # Only one marker, so ID is 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.lifetime = rclpy.duration.Duration(seconds=1.0)  # marker will disappear after 0.1 seconds

        # Calculate the direction.
        direction_angle = self.calculate_direction(IWC_cmd_msg)

        if direction_angle is None:
            self.get_logger().warn("Cannot calculate direction")
            return

        # Set the starting point of the arrow to the robot's current position (or a point slightly above the robot)
        start_point = Point()
        start_point.x = 0.0  # Relative to base_link
        start_point.y = 0.0
        start_point.z = 0.0

        # Calculate the end point of the arrow based on the direction
        end_point = Point()
        arrow_length = 0.5  # Adjust as needed
        end_point.x = arrow_length * np.cos(direction_angle)
        end_point.y = arrow_length * np.sin(direction_angle)
        end_point.z = 0.0

        marker.points.append(start_point)
        marker.points.append(end_point)

        # Set the orientation of the arrow using a quaternion.
        #  We only care about rotation around the Z axis (yaw).
        half_angle = direction_angle / 2.0
        sin_half_angle = np.sin(half_angle)
        cos_half_angle = np.cos(half_angle)
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = sin_half_angle
        marker.pose.orientation.w = cos_half_angle

        marker.scale.x = 0.05  # Shaft diameter
        marker.scale.y = 0.1  # Head diameter
        marker.scale.z = 0.1  # Head length
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # Alpha (transparency)

        self.marker_pub.publish(marker)
        self.get_logger().debug(f"Published marker with direction: {direction_angle:.2f} radians")


def main(args=None):
    rclpy.init(args=args)
    node = RoverTransformBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
