#!/usr/bin/env python3
import numpy as np
from math import degrees, radians, sin, cos, atan2, sqrt, pi, asin

def wrap(chi_1, chi_2):
    while chi_1 - chi_2 > np.pi:
        chi_1 = chi_1 - 2.0 * np.pi
    while chi_1 - chi_2 < -np.pi:
        chi_1 = chi_1 + 2.0 * np.pi
    return chi_1


class GPSCoordinate:
    def __init__(self, lat, lon, alt = 0):
        self.lat = round(lat, 7)
        self.lon = round(lon, 7)
        self.alt = round(alt, 7)

    def __eq__(self, obj):
        return isinstance(obj, GPSCoordinate) and obj.lat == self.lat and obj.lon == self.lon and obj.alt == self.alt

    def __str__(self):
        return "(Lat: {}, Lon: {}, Alt: {})".format(self.lat, self.lon, self.alt)

class GPSTools:
    R = 6369345.0 #Radius of the Earth

    @staticmethod
    def generate_side_waypoint(curr_pose, heading_rad, direction, offset_distance=5.0, offset_angle=0.35):
        """
        Returns a new GPSCoordinate offset slightly to the left or right of current heading,
        using NED heading (0 = North, +pi/2 = East).
        
        Parameters:
            curr_pose: GPSCoordinate object with .lat and .lon
            heading_rad: current heading in radians
            direction: "left" or "right"
            offset_distance: distance in meters
            offset_angle: angle in radians to deviate from current heading (e.g., 0.35 rad ~ 20 deg)

        Returns:
            GPSCoordinate offset in the chosen direction
        """
        if direction == "left":
            new_angle = heading_rad - offset_angle
        elif direction == "right":
            new_angle = heading_rad + offset_angle
        else:
            raise ValueError("Direction must be 'left' or 'right'")

        # Normalize angle to [-pi, pi]
        new_angle = wrap(new_angle, 0.0)

        # Offset in local NED frame (North, East)
        dy = offset_distance * np.cos(new_angle)  # North
        dx = offset_distance * np.sin(new_angle)  # East

        # Convert offset to latitude and longitude
        dlat = (dy / GPSTools.R) * (180.0 / np.pi)
        dlon = (dx / (GPSTools.R * np.cos(np.deg2rad(curr_pose.lat)))) * (180.0 / np.pi)

        offset_lat = curr_pose.lat + dlat
        offset_lon = curr_pose.lon + dlon

        return GPSCoordinate(offset_lat, offset_lon, 0)
    
    @staticmethod
    def get_offset_waypoint(waypoint, initial_heading_rad, direction, offset_distance=4.0):
        """
        Returns a new GPSCoordinate offset to the left or right of the given waypoint,
        using NED heading (0 = North, +pi/2 = East, -pi/2 = West).
        - waypoint: GPSCoordinate (with .lat, .lon)
        - initial_heading_rad: heading from current position to waypoint, in radians [-pi, pi]
        - direction: "left" or "right"
        - offset_distance: meters to offset
        """

        # TODO make this a parmeter how far to offset
        if direction == "left":
            offset_angle = wrap(initial_heading_rad + 0.35, 0.0)
        elif direction == "right":
            offset_angle = wrap(initial_heading_rad - 0.35, 0.0)
        else:
            raise ValueError("Direction must be 'left' or 'right'")

        # Offset in meters (local tangent plane)
        dx = offset_distance * np.cos(offset_angle)
        dy = offset_distance * np.sin(offset_angle)

        # Convert meters to degrees
        dlat = (dy / GPSTools.R) * (180.0 / np.pi)
        dlon = (dx / (GPSTools.R * np.cos(np.deg2rad(waypoint.lat)))) * (180.0 / np.pi)

        offset_lat = waypoint.lat + dlat
        offset_lon = waypoint.lon + dlon

        return GPSCoordinate(offset_lat, offset_lon, 0)
    
    @staticmethod
    def heading_between_lat_lon(point1, point2):
        """
        Returns the heading in radians and degrees between two GPS Coordinates
        :param point1: The origin GPS Coordinate in standard format
        :param point2: The end GPS Coordinate in standard format
        :return: The heading between the GPS Coordinates in radians and degrees
        """
        lat1 = radians(point1.lat)
        lat2 = radians(point2.lat)
        lon1 = radians(point1.lon)
        lon2 = radians(point2.lon)

        deltaLon = lon2 - lon1
        y = sin(deltaLon) * cos(lat2)
        x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(deltaLon)

        rad = atan2(y, x)
        degree = degrees(rad)

        return rad, degree

    @staticmethod
    def heading_distance_to_lat_lon(point1, heading, distance):
        """
        
        :param point1: Origin GPS Coordinate
        :param heading: The heading between the origin GPS Coordinate and the target GPS Coordinate in degrees
        :param distance: The distance between the origin GPS Coordinate and the target GPS Coordinate in meters
        :return: The target GPS Coordinate
        """
        lat1 = radians(point1.lat)
        lon1 = radians(point1.lon)
        heading = radians(heading)

        Radius = GPSTools.R + point1.alt

        lat2 = asin(sin(lat1)*cos(distance/Radius) + cos(lat1) * sin(distance/Radius)*cos(heading))
        lon2 = lon1 + atan2(sin(heading)*sin(distance/Radius)*cos(lat1),
                            cos(distance/Radius)-sin(lat1)*sin(lat2))

        end_coordinate = GPSCoordinate(degrees(lat2), degrees(lon2), point1.alt)
        return end_coordinate

    @staticmethod
    def distance_between_lat_lon(point1, point2):
        """

        :param point1: The origin GPS Coordinate in standard format
        :param point2: The end GPS Coordinate in standard format
        :return: The distance between the origin GPS Coordinate and the target GPS Coordinate in meters
        """
        lat1 = radians(point1.lat)
        lat2 = radians(point2.lat)
        lon1 = radians(point1.lon)
        lon2 = radians(point2.lon)

        deltaLon = lon2 - lon1
        deltaLat = lat2 - lat1

        a = sin(deltaLat / 2) ** 2 + cos(lat1) * cos(lat2) * sin(deltaLon / 2) ** 2  # Haversine Formula
        c = 2 * atan2(sqrt(a), sqrt(1 - a))  # radians
        distance = (GPSTools.R + point1.alt) * c
        return distance

    @staticmethod
    def north_east_distance_between_lat_lon(point1, point2):
        """

        :param point1: The origin GPS Coordinate in standard format
        :param point2: The end GPS Coordinate in standard format
        :return: The distance north and distance east between the origin and the target GPS Coordinate in meters
        """
        distance = GPSTools.distance_between_lat_lon(point1, point2)
        heading = GPSTools.heading_between_lat_lon(point1, point2)

        distance_north = distance * cos(heading[0])
        distance_east = distance * sin(heading[0])

        return distance_north, distance_east

    @staticmethod
    def x_y_to_lat_lon(point1, x, y, heading_offset=0):
        """

        :param point1: The origin GPS Coordinate in standard format
        :param x: The distance in the east (x) direction of the target GPS Coordinate in meters
        :param y: The distance in the north (y) direction of the target GPS Coordinate in meters
        :return:
        """
        distance = sqrt(x**2 + y**2)
        heading = degrees(asin(y/distance)) + heading_offset
        end_coordinate = GPSTools.heading_distance_to_lat_lon(point1, heading, distance)
        return end_coordinate

    @staticmethod
    def midpoint_from_lat_lon(point1, point2):
        lat1 = radians(point1.lat)
        lat2 = radians(point2.lat)
        lon1 = radians(point1.lon)
        lon2 = radians(point2.lon)

        deltaLon = radians(lon2 - lon1)

        Bx = cos(lat2) * cos(deltaLon)
        By = cos(lat2) * sin(deltaLon)
        lat3 = atan2(sin(lat1) + sin(lat2), sqrt((cos(lat1) + Bx)**2 + By**2))
        # lat3 = atan2(sin(lat1) + sin(lat2), sqrt((cos(lat1) + Bx) * (cos(lat1) + Bx) + By * By))
        lon3 = lon1 + atan2(By, cos(lat1) + Bx)
        lon3 = (lon3 + 540) % 360 - 180
        midpoint_coordinate = GPSCoordinate(degrees(lat3), degrees(lon3))
        return midpoint_coordinate

    @staticmethod
    def median_of_lat_lon_list(points):
        if len(points) == 0:
            print("No waypoints in list")
            return
        distances = []
        for point in points:
            distances.append(GPSTools.sum_distance_to_points(point, points))
        min_index = distances.index(min(distances))
        median_coordinate = points[min_index]

        return median_coordinate

    @staticmethod
    def sum_distance_to_points(origin, points):
        sum_distance = 0
        for point in points:
            sum_distance += GPSTools.distance_between_lat_lon(origin, point)
        return sum_distance

