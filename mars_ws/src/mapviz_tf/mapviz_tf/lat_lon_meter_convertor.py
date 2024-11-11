#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from math import cos, radians


class LatLonConvertor(Node):
    def __init__(self):
        super().__init__("lat_lon_converter")

        # # Initialize the node
        # self.declare_parameter("map_origin_index", 0)
        # self.declare_parameter("initialize_origin/local_xy_origins", [])

        # Use os.getenv to fetch MAPVIZ_LOCATION or default to 'hanksville'
        self.location = os.getenv('MAPVIZ_LOCATION', 'hanksville')


        # self.declare_parameter(latitude_param, 0.0)  # Declare with a default

        # Access the altitude parameter for the selected location
        latitude_param = f'locations.{self.location}.latitude'
        self.latitude = self.get_parameter(latitude_param).value
        self.get_logger().info(f"{self.location.capitalize()} Latitude: {self.latitude}")

        longitude_param = f'locations.{self.location}.longitude'
        self.longitude = self.get_parameter(longitude_param).value
        self.get_logger().info(f"{self.location.capitalize()} Longitude: {self.longitude}")

        # Fetch the origin point from parameters.
        # index = self.get_parameter("map_origin_index").value
        # origins = self.get_parameter("initialize_origin/local_xy_origins").value
        # if index >= len(origins):
        #     self.get_logger().error("Index out of bounds for local_xy_origins list")
        #     return

        # self.origin = origins[index]
        self.set_meters_per_degree_lat_lon()

    def set_meters_per_degree_lat_lon(self):
        # Calculate meters per degree lat/lon.
        self.meters_per_degree_latitude = (
            111132.92
            - 559.82 * cos(radians(2 * self.latitude))
            + 1.175 * cos(radians(4 * self.latitude))
            - 0.0023 * cos(radians(6 * self.latitude))
        )

        self.meters_per_degree_longitude = (
            111412.84 * cos(radians(self.latitude))
            - 93.5 * cos(radians(3 * self.latitude))
            + 0.118 * cos(radians(5 * self.latitude))
        )

    def get_meters_per_degree_lat_lon(self):
        """
        Returns the meter-per-degree factor.
        """
        return (self.meters_per_degree_latitude, self.meters_per_degree_longitude)

    # def get_origin(self):
    #     """
    #     Returns the origin point.
    #     """
    #     self.get_logger().info(
    #         f"ORIGIN SET TO "{self.location.capitalize()}": [Latitude: {self.latitude}, Longitude: {self.longitude}]"
    #     )
    #     return self.origin

    def convert_to_meters(self, lat, lon):
        """
        Converts latitude and longitude coordinates to meters.
        """
        coordinates = {
            "x": (lon - self.longitude) * self.meters_per_degree_longitude,
            "y": (lat - self.latitude) * self.meters_per_degree_latitude,
        }
        return coordinates

    def convert_to_latlon(self, x, y):
        """
        Converts meters coordinates to latitude and longitude.
        """
        coordinates = {
            "lat": (y / self.meters_per_degree_latitude) + self.latitude,
            "lon": (x / self.meters_per_degree_longitude) + self.longitude,
        }
        return coordinates


def main(args=None):
    rclpy.init(args=args)
    node = LatLonConvertor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
