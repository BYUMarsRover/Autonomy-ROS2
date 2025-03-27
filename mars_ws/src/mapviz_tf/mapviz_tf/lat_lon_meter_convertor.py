#!/usr/bin/env python3
import os
import yaml
from math import cos, radians


class LatLonConvertor:
    def __init__(self):
        # Use os.getenv to fetch MAPVIZ_LOCATION or default to 'hanksville'
        self.location = os.getenv('MAPVIZ_LOCATION', 'hanksville')
        mapviz_origins_path = os.path.join(os.getenv('HOME', '/home/marsrover'),'mars_ws/src/mapviz_tf/params/mapviz_origins.yaml')
        with open(mapviz_origins_path, 'r') as file:
            config = yaml.safe_load(file)
            self.latitude = config.get('locations', {}).get(self.location, {}).get('latitude', 0.0)
            self.longitude = config.get('locations', {}).get(self.location, {}).get('longitude', 0.0)
        # print(f"{self.location.capitalize()} Latitude: {self.latitude}")
        # print(f"{self.location.capitalize()} Longitude: {self.longitude}")
        # Set the meter-per-degree conversion factors based on the latitude
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
        Returns the meter-per-degree factor for latitude and longitude.
        """
        return (self.meters_per_degree_latitude, self.meters_per_degree_longitude)

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
    
