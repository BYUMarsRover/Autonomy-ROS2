#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import numpy as np
import math
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, Imu
from visualization_msgs.msg import Marker
from rover_msgs.msg import IWCMotors
from std_msgs.msg import Float32
from mapviz_tf.lat_lon_meter_convertor import LatLonConvertor
from rover_msgs.msg import RoverStateSingleton
from builtin_interfaces.msg import Duration
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


        # State input
        self.odom_sub = self.create_subscription(RoverStateSingleton, "/odometry/rover_state_singleton", self.odom_callback, 10)
        
        # Motor command Input
        self.IWC_sub = self.create_subscription(IWCMotors, '/IWC_motorControl', self.publish_direction_marker, 1)
        self.marker_pub = self.create_publisher(Marker, 'mapviz_markers', 10)  # Added marker publisher

        # Roll and Pitch Display
        self.imu_sub = self.create_subscription(Imu, '/zed/zed_node/imu/data', self.roll_pitch_display, 10)
        self.roll_pub = self.create_publisher(Float32, 'roll_float', 1)
        self.pitch_pub = self.create_publisher(Float32, 'pitch_float', 1)
        # Low-pass filter parameters
        self.alpha = 0.06  # adjust between 0 (super smooth) and 1 (no filtering)
        self.filtered_roll = 0.0
        self.filtered_pitch = 0.0


    def roll_pitch_display(self, msg):
        # Get orientation quaternion
        q = msg.orientation

        # Normalize quaternion
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        x = q.x / norm
        y = q.y / norm
        z = q.z / norm
        w = q.w / norm

        # Calculate roll
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Calculate pitch
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)


        # Low-pass filter
        self.filtered_roll = self.alpha * roll + (1 - self.alpha) * self.filtered_roll
        self.filtered_pitch = self.alpha * pitch + (1 - self.alpha) * self.filtered_pitch

        # Convert to degrees and round 
        roll_deg = np.clip(np.round(np.rad2deg(self.filtered_roll), 2), -90.0, 90.0)
        pitch_deg = np.clip(np.round(np.rad2deg(self.filtered_pitch), 2), -90.0, 90.0)


        # Publish filtered roll and pitch
        roll_msg = Float32()
        roll_msg.data = roll_deg
        self.roll_pub.publish(roll_msg)

        pitch_msg = Float32()
        pitch_msg.data = pitch_deg
        self.pitch_pub.publish(pitch_msg)


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
        Calculates the intended angular and linear velocity of the robot based on the motor commands.

        Args:
            IWC_cmd_msg (IWCMotors): The motor command message.

        """
        # Extract wheel speeds and directions
        # left_front_speed = IWC_cmd_msg.left_front_speed
        # left_front_dir = IWC_cmd_msg.left_front_dir
        left_middle_speed = IWC_cmd_msg.left_middle_speed
        left_middle_dir = IWC_cmd_msg.left_middle_dir
        # left_rear_speed = IWC_cmd_msg.left_rear_speed
        # left_rear_dir = IWC_cmd_msg.left_rear_dir

        # right_front_speed = IWC_cmd_msg.right_front_speed
        # right_front_dir = IWC_cmd_msg.right_front_dir
        right_middle_speed = IWC_cmd_msg.right_middle_speed
        right_middle_dir = IWC_cmd_msg.right_middle_dir
        # right_rear_speed = IWC_cmd_msg.right_rear_speed
        # right_rear_dir = IWC_cmd_msg.right_rear_dir

        # Convert speeds and directions to signed velocities.  Forward is positive.
        # left_front_velocity = left_front_speed if left_front_dir else -left_front_speed
        left_middle_velocity = left_middle_speed if left_middle_dir else -left_middle_speed
        # left_rear_velocity = left_rear_speed if left_rear_dir else -left_rear_speed

        # right_front_velocity = right_front_speed if right_front_dir else -right_front_speed
        right_middle_velocity = right_middle_speed if right_middle_dir else -right_middle_speed
        # right_rear_velocity = right_rear_speed if right_rear_dir else -right_rear_speed

        # Simple model: Average left and right side velocities.
        # left_velocity = (left_front_velocity + left_middle_velocity + left_rear_velocity) / 3.0
        # right_velocity = (right_front_velocity + right_middle_velocity + right_rear_velocity) / 3.0

        # Handle the case where both sides have the same velocity (straight line)
        velocity = (left_middle_velocity + right_middle_velocity) / 2.0
        omega = (left_middle_velocity - right_middle_velocity) / 2.0  # IN NED frame which is being used right now I believe
        # self.get_logger().info(f"left velocity: {left_middle_velocity}, right: {right_middle_velocity}")

        return omega, velocity


    def publish_direction_marker(self, IWC_cmd_msg):
        """Publishes a visualization_msgs/Marker arrow indicating the robot's intended direction."""
        marker = Marker()
        marker.header.frame_id = "rover"  # Or "odom" or whatever your robot's base frame is
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "Motor Control"
        marker.id = 0  # Only one marker, so ID is 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.lifetime = Duration(sec=1, nanosec=0)  # marker will disappear after 0.1 seconds

        # Calculate the direction.
        omega, velocity = self.calculate_direction(IWC_cmd_msg)

        angle = omega * ((np.pi /2.0 )/ 255.0) # Angle in radians

        # Set the starting point of the arrow to the robot's current position (or a point slightly above the robot)
        start_point = Point()
        start_point.x = 0.0  # Relative to base_link
        start_point.y = 0.0
        start_point.z = 0.0

        # Calculate the end point of the arrow based on the direction
        end_point = Point()
        arrow_length = 8.5  # Adjust as needed

        # Rover image is oriented with Y axis forward
        end_point.y = arrow_length * np.cos(angle)
        end_point.x = arrow_length * np.sin(angle)
        end_point.z = 0.0

        marker.points.append(start_point)
        marker.points.append(end_point)
        
        # DO WE NEED THE ORIENTATION WHEN THE END POINT IS ALREADY SET?
        # Set the orientation of the arrow using a quaternion.
        #  We only care about rotation around the Z axis (yaw).
        # half_angle = np.pi / -2.0
        # sin_half_angle = np.sin(half_angle)
        # cos_half_angle = np.cos(half_angle)
        # marker.pose.orientation.x = 0.0
        # marker.pose.orientation.y = 0.0
        # # marker.pose.orientation.z = 0.0
        # # marker.pose.orientation.w = 0.0
        # marker.pose.orientation.z = sin_half_angle
        # marker.pose.orientation.w = cos_half_angle

        marker.scale.x = 6.0  # Shaft diameter
        marker.scale.y = 7.0  # Head diameter
        marker.scale.z = 6.0  # Head length
        marker.color.r = 1 - (velocity / 255)
        marker.color.g = velocity / 255.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # Alpha (transparency)

        self.marker_pub.publish(marker)
        # self.get_logger().info(f"Published marker with direction: {angle:.2f} radians, and velocity: {velocity}")


def main(args=None):
    rclpy.init(args=args)
    node = RoverTransformBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
