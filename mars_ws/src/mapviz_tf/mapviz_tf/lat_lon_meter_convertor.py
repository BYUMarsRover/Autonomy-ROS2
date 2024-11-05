#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from math import cos, radians


class LatLonConvertor(Node):
    def __init__(self):
        super().__init__("lat_lon_converter")

        # Initialize the node
        self.declare_parameter("map_origin_index", 0)
        self.declare_parameter("initialize_origin/local_xy_origins", [])

        # Fetch the origin point from parameters.
        index = self.get_parameter("map_origin_index").value
        origins = self.get_parameter("initialize_origin/local_xy_origins").value
        if index >= len(origins):
            self.get_logger().error("Index out of bounds for local_xy_origins list")
            return

        self.origin = origins[index]
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
        Returns the meter-per-degree factor.
        """
        return (self.meters_per_degree_latitude, self.meters_per_degree_longitude)

    def get_origin(self):
        """
        Returns the origin point.
        """
        self.get_logger().info(
            f'ORIGIN SET TO "{self.origin["name"]}": [{self.origin["latitude"]}, {self.origin["longitude"]}]'
        )
        return self.origin

    def convert_to_meters(self, lat, lon):
        """
        Converts latitude and longitude coordinates to meters.
        """
        coordinates = {
            "x": (lon - self.origin["longitude"]) * self.meters_per_degree_longitude,
            "y": (lat - self.origin["latitude"]) * self.meters_per_degree_latitude,
        }
        return coordinates

    def convert_to_latlon(self, x, y):
        """
        Converts meters coordinates to latitude and longitude.
        """
        coordinates = {
            "lat": (y / self.meters_per_degree_latitude) + self.origin["latitude"],
            "lon": (x / self.meters_per_degree_longitude) + self.origin["longitude"],
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
