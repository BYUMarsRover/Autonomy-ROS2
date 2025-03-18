#!/usr/bin/env python3

import numpy as np
import random
import rclpy
from rclpy.node import Node
import os

from rover_msgs.msg import RoverStateSingleton
from sensor_msgs.msg import NavSatFix


ROS_RATE = 10.0


HANK_LAT=38.406441
HANK_LONG=-110.791932

HANK_AUT_2024_COMP_LAT=38.423216
HANK_AUT_2024_COMP_LONG=-110.786483

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

LAT_INIT= HANK_AUT_2024_COMP_LAT
LONG_INIT= HANK_AUT_2024_COMP_LONG

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

        self.singleton_publisher = self.create_publisher(RoverStateSingleton, '/odometry/rover_state_singleton', 10)  # Publishes the singleton message
        
        self.timer = self.create_timer(1.0 / ROS_RATE, self.publish_singleton_data) # Publish at 15Hz

        self.ll_x = np.linspace(0, 2*np.pi, num=LL_PRECISION)
        self.deg = np.linspace(0, 360, num=HEADING_PRECISION) # heading
        self.latitude = LAT_INIT
        self.longitude = LONG_INIT
        print("LAT_INIT: ", LAT_INIT)
        print("LONG_INIT: ", LONG_INIT)

        self.latitude_unfilt = LAT_INIT
        self.longitude_unfilt = LONG_INIT

        self.ll_counter = Counter(LL_PRECISION)
        self.heading_counter = Counter(HEADING_PRECISION)


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
        # self._inc_counters()

        # self.filter_gps.latitude, self.filter_gps.longitude = self._generate_lat_long(
        #     self.ll_counter)
        # self.gps.latitude, self.gps.longitude = self._generate_unfiltered_lat_long(
        #     self.ll_counter)
        # self.map_yaw = self._generate_heading(self.heading_counter)

        # Make the rover move in a circle around the init location
        # if not hasattr(self, 'angle'):
        #     self.angle = 0 # North East Down (measured from North cw is positive)
        # self.angle += 0.4
        # if self.angle >= 360:
        #     self.angle = 0
        # self.latitude, self.longitude = self.rotate()

        # Make the rover spin in place
        self.map_yaw -= 0.3 # -0.1 * ROS_RATE degrees per second

        # Wrap
        if self.map_yaw < -180:
            self.map_yaw += 360

        self.gps.latitude = self.latitude
        self.gps.longitude = self.longitude
        self.filter_gps.latitude = self.latitude
        self.filter_gps.longitude = self.longitude

        msg = RoverStateSingleton(
            map_yaw=self.map_yaw,
            gps=self.gps,
            filter_gps=self.filter_gps,
        )
        self.singleton_publisher.publish(msg)

    def rotate(self):
        R = 6378137  # Earth's radius in meters
        d = 30  # Radius of rotation in meters

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

    def _inc_counters(self):
        self.ll_counter = self._inc_counter(self.ll_counter)
        self.heading_counter = self._inc_counter(self.heading_counter)

    def _inc_counter(self, counter):
        counter.count += 1
        if counter.count == counter.max:
            counter.count = 0
        return counter

    def _generate_lat_long(self, ll_counter):
        i = ll_counter.count

        # very small difference from true lat/long. This shows how cool and precise
        # RTK GPS can be.
        rtk_precison = 0.01 / METERS_PER_DEGREE # 0.01 meters in degrees

        # not sure if this math is right, but it looks pretty good on a map
        self.latitude = self.latitude + \
            ((np.sin(self.ll_x[i] + np.pi/2) * CIRCLE_RADIUS) / (METERS_PER_DEGREE * (2*np.pi))) + \
                random.uniform(-rtk_precison, rtk_precison)

        self.longitude = self.longitude + \
            ((np.sin(self.ll_x[i]) * CIRCLE_RADIUS) / (METERS_PER_DEGREE * (2*np.pi))) + \
                random.uniform(-rtk_precison, rtk_precison)

        return self.latitude, self.longitude

    def _generate_unfiltered_lat_long(self, ll_counter):
        i = ll_counter.count

        random_offset = 5 / METERS_PER_DEGREE

        self.latitude_unfilt = self.latitude + \
            ((np.sin(self.ll_x[i] + np.pi/2) * CIRCLE_RADIUS) / (METERS_PER_DEGREE * (2*np.pi))) + \
                random.uniform(-random_offset, random_offset)

        self.longitude_unfilt = self.longitude + \
            ((np.sin(self.ll_x[i]) * CIRCLE_RADIUS) / (METERS_PER_DEGREE * (2*np.pi))) + \
                random.uniform(-random_offset, random_offset)

        return self.latitude_unfilt, self.longitude_unfilt

    def _generate_heading(self, heading_counter):
        i = heading_counter.count

        return self.deg[i]
    
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
