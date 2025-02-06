#!/usr/bin/python3

import math
import rclpy
from rclpy.node import Node

# Monkey patch (there is an bug in the transforms3d library within tf_transformations that uses np.float instead of float)
import numpy as np
np.float = float  # Temporary alias for compatibility

from tf_transformations import euler_from_quaternion

from nav_msgs.msg import Odometry
from rover_msgs.msg import RoverStateSingleton
from sensor_msgs.msg import NavSatFix, Imu
import numpy as np

np.float = float # Bug fix hack

class RoverStateSingletonCreator(Node):
    """
    This class subscribes to gps data and zed orientation data for lat/lon and zed_yaw
    """
    def __init__(self):
        super().__init__('rover_state_singleton_creator')

        # Subscribers
        # TODO: self.zed_orientation_subscription = self.create_subscription(Imu, "[ORIENTATION TOPIC FROM ZED]", self.convert_map, 10)  # Subscribes to the ZED orientation data

        self.filtered_gps_subscription = self.create_subscription(NavSatFix, "/gps/filtered", self.convert_filtered_gps, 10)  # Subscribes to the filtered GPS data from the UKF output
        self.gps_subscription = self.create_subscription(NavSatFix, "/ins/lla", self.convert_gps, 10) # Subscribes to unfiltered GPS data

        # Publishers
        self.singleton_publisher = self.create_publisher(RoverStateSingleton, '/odometry/rover_state_singleton', 10) # Publishes the singleton message

        self.map_roll = 0.0
        self.map_pitch = 0.0
        self.map_yaw = 0.0

        self.odom_roll = 0.0
        self.odom_pitch = 0.0
        self.odom_yaw = 0.0

        self.map_x = 0.0
        self.map_y = 0.0
        self.map_z = 0.0

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_z = 0.0

        self.map_x_dot = 0.0
        self.map_y_dot = 0.0
        self.map_z_dot = 0.0

        self.odom_x_dot = 0.0
        self.odom_y_dot = 0.0
        self.odom_z_dot = 0.0

        self.map_roll_dot = 0.0
        self.map_pitch_dot = 0.0
        self.map_yaw_dot = 0.0

        self.odom_roll_dot = 0.0
        self.odom_pitch_dot = 0.0
        self.odom_yaw_dot = 0.0

        self.gps = NavSatFix()
        self.filter_gps = NavSatFix()

    def convert_map(self, message):
        """
        Converts orientation from quaternions to euler (converts them into human readable roll, pitch, yaw) angles and sets the map variables
        """
        orientation_q = message.orientation  # Extracts the orientation data from the message
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]  # Extracts quaternion data
        euler = euler_from_quaternion(orientation_list)  # Transforms the quaternion data into euler angles

        # sets the roll, pitch and yaw based on the euler angles and converts them to degrees
        zed_roll = euler[0] * 180/math.pi
        zed_pitch = euler[1] * 180/math.pi
        zed_yaw = euler[2] * 180/math.pi

        # TODO: Convert from ZED frame to Map Frame

        self.map_roll = euler[0] * 180/math.pi
        self.map_pitch = euler[1] * 180/math.pi
        self.map_yaw = euler[2] * 180/math.pi 

        self.publish_message()


    def convert_odom(self, message):
        """
        Converts orientation from quaternions to euler (converts them into human readable roll, pitch, yaw) angles and sets the odom variables
        """
        orientation_q = message.pose.pose.orientation  # Extracts the orientation data from the message
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]  # Extracts quaternion data
        euler = euler_from_quaternion(orientation_list)  # Transforms the quaternion data into euler angles

        # sets the roll, pitch and yaw based on the euler angles and converts them to degrees
        self.odom_roll = euler[0] * 180/math.pi
        self.odom_pitch = euler[1] * 180/math.pi
        self.odom_yaw = euler[2] * 180/math.pi

        self.odom_x = message.pose.pose.position.x
        self.odom_y = message.pose.pose.position.y
        self.odom_z = message.pose.pose.position.z

        self.odom_x_dot = message.twist.twist.linear.x
        self.odom_y_dot = message.twist.twist.linear.y
        self.odom_z_dot = message.twist.twist.linear.z

        self.odom_roll_dot = message.twist.twist.angular.x * 180/math.pi
        self.odom_pitch_dot = message.twist.twist.angular.y * 180/math.pi
        self.odom_yaw_dot = message.twist.twist.angular.z * 180/math.pi

        self.publish_message()

    def convert_filtered_gps(self, message):
        """
        Sets the filter GPS variable
        """
        self.filter_gps = message
        self.publish_message()

    def convert_gps(self, message):
        """
        Sets the GPS variable
        """
        self.gps = message
        self.publish_message()

    def publish_message(self):
        """
        Creates and publishes the singleton message
        """
        message = RoverStateSingleton(
            map_roll = self.map_roll,
            map_pitch = self.map_pitch,
            map_yaw = self.map_yaw,
            odom_roll = self.odom_roll,
            odom_pitch = self.odom_pitch,
            odom_yaw =self.odom_yaw,
            map_x = self.map_x,
            map_y = self.map_y,
            map_z = self.map_z,
            odom_x = self.odom_x,
            odom_y = self.odom_y,
            odom_z = self.odom_z,
            map_x_dot = self.map_x_dot,
            map_y_dot = self.map_y_dot,
            map_z_dot = self.map_z_dot,
            odom_x_dot = self.odom_x_dot,
            odom_y_dot = self.odom_y_dot,
            odom_z_dot = self.odom_z_dot,
            map_roll_dot = self.map_roll_dot,
            map_pitch_dot = self.map_pitch_dot,
            map_yaw_dot = self.map_yaw_dot,
            odom_roll_dot = self.odom_roll_dot,
            odom_pitch_dot = self.odom_pitch_dot,
            odom_yaw_dot = self.odom_yaw_dot,
            gps = self.gps,
            filter_gps = self.filter_gps
        )
        self.singleton_publisher.publish(message)

def main(args=None):
    rclpy.init(args=args)
    rover_state_singleton_creator = RoverStateSingletonCreator()
    rclpy.spin(rover_state_singleton_creator)
    rover_state_singleton_creator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
