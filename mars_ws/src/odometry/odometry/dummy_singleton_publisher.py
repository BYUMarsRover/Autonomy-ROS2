#!/usr/bin/env python3

import numpy as np
import random
import rclpy
from rclpy.node import Node
import os

from rover_msgs.msg import RoverStateSingleton, IWCMotors
from sensor_msgs.msg import NavSatFix


ROS_RATE = 10.0


HANK_LAT=38.406441
HANK_LONG=-110.791932

HANK_AUT_2024_COMP_LAT=38.423216
HANK_AUT_2024_COMP_LONG=-110.786483

HANK_AUT_2025_COMP_LAT=38.406441
HANK_AUT_2025_COMP_LONG=-110.791932

GRAVEL_LAT=40.322415
GRAVEL_LONG=-111.64317

BYU_LAT=40.2497218
BYU_LONG=-111.649276

ROCK_LAT=40.267147
ROCK_LONG=-111.632455

# LAT_INIT=HANK_LAT
# LONG_INIT=HANK_LONG

# LAT_INIT=GRAVEL_LAT
# LONG_INIT=GRAVEL_LONG

# LAT_INIT= HANK_AUT_2024_COMP_LAT
# LAT_INIT= HANK_AUT_2024_COMP_LAT

LONG_INIT= HANK_AUT_2025_COMP_LONG
LONG_INIT= HANK_AUT_2025_COMP_LONG

# LAT_INIT=BYU_LAT
# LONG_INIT=BYU_LONG

LL_PRECISION=100
HEADING_PRECISION=LL_PRECISION

METERS_PER_DEGREE = 111139.954
CIRCLE_RADIUS = 5 # meters
class Counter():
    def __init__(self, max):
        self.count = 0
        self.max = max

class DummySingletonPublisher(Node):
    def __init__(self):
        super().__init__('dummy_singleton_publisher')

        # Publishers
        self.singleton_publisher = self.create_publisher(RoverStateSingleton, '/odometry/rover_state_singleton', 10)  # Publishes the singleton message
        self.timer = self.create_timer(1.0 / ROS_RATE, self.publish_singleton_data) # Publish at 15Hz

        # Subscribers
        # NOTE: Uncomment this to simulate the rover moving according to the motor commands
        self.create_subscription(IWCMotors, '/mobility/auto_drive_cmds', self.motor_cmd_callback, 10)

        self.latitude = LAT_INIT
        self.longitude = LONG_INIT
        print("LAT_INIT: ", LAT_INIT)
        print("LONG_INIT: ", LONG_INIT)

        self.latitude_unfilt = LAT_INIT
        self.longitude_unfilt = LONG_INIT


        self.map_roll = 0
        self.map_pitch = 0
        self.map_yaw = 0.0

        self.odom_roll = 0
        self.odom_pitch = 0
        self.odom_yaw = 0

        self.map_x = 0
        self.map_y = 0
        self.map_z = 0

        self.odom_x = 0
        self.odom_y = 0
        self.odom_z = 0

        self.map_x_dot = 0
        self.map_y_dot = 0
        self.map_z_dot = 0

        self.odom_x_dot = 0
        self.odom_y_dot = 0
        self.odom_z_dot = 0

        self.map_roll_dot = 0
        self.map_pitch_dot = 0
        self.map_yaw_dot = 0

        self.odom_roll_dot = 0
        self.odom_pitch_dot = 0
        self.odom_yaw_dot = 0

        self.gps = NavSatFix()
        self.filter_gps = NavSatFix()

    def publish_singleton_data(self):

        # Simulate Driving


        # Make the rover move in a circle around the init location
        # if not hasattr(self, 'angle'):
        #     self.angle = 0 # North East Down (measured from North cw is positive)
        # self.angle += 1.0
        # if self.angle >= 360:
        #     self.angle = 0
        # self.latitude, self.longitude = self.rotate()

        # Make the rover spin in place
        # self.map_yaw += 0.3 # -0.1 * ROS_RATE degrees per second

        # Loop for testing
        # if self.map_yaw > 90.0:
        #     self.map_yaw = 0.0

        # Wrap
        if self.map_yaw < -180:
            self.map_yaw += 360
        elif self.map_yaw > 180:
            self.map_yaw -=360

        self.gps.latitude = self.latitude + (np.random.rand() - 1)*0.0000005
        self.gps.longitude = self.longitude + (np.random.rand() - 1)*0.0000005
        self.filter_gps.latitude = self.latitude
        self.filter_gps.longitude = self.longitude

        # self.gps.latitude = LAT_INIT + (np.random.rand() - 1)*0.00002
        # self.gps.longitude = LONG_INIT + (np.random.rand() - 1)*0.00002

        msg = RoverStateSingleton(
            map_yaw=self.map_yaw,
            gps=self.gps,
            filter_gps=self.filter_gps,
        )
        self.singleton_publisher.publish(msg)

    def motor_cmd_callback(self, msg):
        # update the rover yaw and gps based on the motor commands
        rv = msg.right_front_speed/255.0
        if msg.right_front_dir == 0:
            rv = -rv

        lv = msg.left_front_speed/255.0
        if msg.left_front_dir == 0:
            lv = -lv

        C_vel = 0.1
        v = (lv + rv)**2*C_vel

        dlat, dlon = self.compute_delta_lat_lon(self.latitude, self.longitude, v, self.map_yaw)
        self.latitude += dlat
        self.longitude += dlon

        C_yaw = 1.0
        dyaw = (lv - rv) * C_yaw
        self.map_yaw += dyaw

        return


    def rotate(self):
        R = 6378137  # Earth's radius in meters
        d = 50  # Radius of rotation in meters

        # Convert angle to radians
        angle_radians = np.deg2rad(self.angle)

        # Calculate the latitude and longitude offsets
        delta_lat = d*np.cos(angle_radians)/R
        delta_lon = d*np.sin(angle_radians)/R

        # Calculate new latitude and longitude
        new_lat = LAT_INIT + np.rad2deg(delta_lat)
        new_lon = LONG_INIT + np.rad2deg(delta_lon)

        # self.get_logger().info(f'lat: {new_lat}, lon: {new_lon}')

        return new_lat, new_lon
    
    def compute_delta_lat_lon(self, lat, lon, velocity, angle, time_step=0.5):
        """
        Compute the change in latitude and longitude given an initial GPS location,
        velocity (m/s), and heading angle (degrees from north).

        :param lat: Initial latitude in degrees
        :param lon: Initial longitude in degrees
        :param velocity: Velocity in meters per second
        :param angle: Heading angle in degrees from north (0° = North, 90° = East)
        :param time_step: Time duration in seconds (default is 1s)
        :return: (delta_lat, delta_lon) in degrees
        """
        # Convert latitude and longitude to radians
        lat_rad = np.deg2rad(lat)
        lon_rad = np.deg2rad(lon)

        # Convert angle to radians
        angle_rad = np.deg2rad(angle)

        # Earth's radius in meters
        R = 6378137  # WGS-84 approximation

        # Compute distance traveled in the given time step
        distance = velocity * time_step

        # Compute delta latitude in degrees
        delta_lat = (distance * np.cos(angle_rad)) / R
        delta_lat_deg = np.rad2deg(delta_lat)

        # Compute delta longitude in degrees (adjusted for latitude)
        delta_lon = (distance * np.sin(angle_rad)) / (R * np.cos(lat_rad))
        delta_lon_deg = np.rad2deg(delta_lon)

        return delta_lat_deg, delta_lon_deg
    
def main():
    rclpy.init()

    location = os.getenv("MAPVIZ_LOCATION")
    print(location)
    if location == "hanksville":
        LAT_INIT=HANK_LAT
        LONG_INIT=HANK_LONG
    elif location == "byu":
        LAT_INIT=BYU_LAT
        LONG_INIT=BYU_LONG
    elif location == "gravel_pit":
        LAT_INIT=GRAVEL_LAT
        LONG_INIT=GRAVEL_LONG
    elif location == "rock_canyon":
        LAT_INIT=ROCK_LAT
        LONG_INIT=ROCK_LONG
    else:
        LAT_INIT=HANK_LAT
        LONG_INIT=HANK_LONG

    dummy_singleton_pub = DummySingletonPublisher()
    rclpy.spin(dummy_singleton_pub)

    dummy_singleton_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
