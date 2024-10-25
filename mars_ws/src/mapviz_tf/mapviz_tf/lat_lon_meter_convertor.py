#!/usr/bin/env python3
import rospy
from math import cos, radians

# Utility Class
class LatLonConvertor:
    def __init__(self):
        self.origin = {}

        # Get the origin point from the launch file parameters.
        index = rospy.get_param("map_origin_index")
        self.origin = rospy.get_param("/initialize_origin/local_xy_origins")[index]
        self.set_meters_per_degree_lat_lon()

    def set_meters_per_degree_lat_lon(self):
        # Calculate meters per degree lat/lon.
        self.meters_per_degree_latitude = (
            111132.92
            - 559.82 * cos(radians(2 * self.origin["latitude"]))
            + 1.175 * cos(radians(4 * self.origin["latitude"]))
            - 0.0023 * cos(radians(6 * self.origin["latitude"]))
        )

        self.meters_per_degree_longitude = (
            111412.84 * cos(radians(self.origin["latitude"]))
            - 93.5 * cos(radians(3 * self.origin["latitude"]))
            + 0.118 * cos(radians(5 * self.origin["latitude"]))
        )

    def get_meters_per_degree_lat_lon(self):
        """
        This returns the meter_per_degree factor.
        """
        return (self.meters_per_degree_latitude, self.meters_per_degree_longitude)

    def get_origin(self):
        """
        This returns the origin point.
        """

        rospy.loginfo(
            'ORIGIN SET TO "{}": [{}, {}]'.format(
                self.origin["name"], self.origin["latitude"], self.origin["longitude"]
            )
        )

        return self.origin

    def convert_to_meters(self, lat, lon):
        """
        This Converts the lat lon coordinates to meters coordinates.
        """
        coordinates = {
            "x": (lon - self.origin["longitude"]) * self.meters_per_degree_longitude,
            "y": (lat - self.origin["latitude"]) * self.meters_per_degree_latitude,
        }
        return coordinates

    def convert_to_latlon(self, x, y):
        """
        This Converts the meters coordinates to lat lon coordinates.
        """
        coordinates = {
            "lat": (y / self.meters_per_degree_latitude) + self.origin["latitude"],
            "lon": (x / self.meters_per_degree_longitude) + self.origin["longitude"],
        }
        return coordinates
