# Planner visualization without mapviz origin logic

from autonomy.utils.gps_utils import (
    latLonYaw2Geopose,
    meters2LatLon,
    latLon2Meters,
)
from autonomy.utils.plan_utils import (
    basicPathPlanner,
    basicOrderPlanner,
)
from autonomy.utils.terrain_utils import (
    terrainPathPlanner,
    terrainOrderPlanner,
)

import rasterio
from rasterio.transform import rowcol
import matplotlib.pyplot as plt
import numpy as np
import utm

class Planner:
    def __init__(self, tiff_path):
        # Configurable parameters
        self.use_terrain_path_planner = True
        self.use_terrain_order_planner = False
        self.elevation_cost = 200
        self.waypoint_distance = 2.0
        self.elevation_limit = 0.35
        self.roll_cost = 0.0
        self.roll_limit = 100.0

        self.margin = 10  # pixels

        # Load raster image
        self.tiff_path = tiff_path
        with rasterio.open(self.tiff_path) as src:
            self.full_image = src.read(1)
            self.transform = src.transform

        print("TIFF loaded.")

    def latlon_to_pixel(self, lat, lon):
        x, y, _, _ = utm.from_latlon(lat, lon)
        row, col = rowcol(self.transform, x, y)
        crop_row = row - self.crop_origin[0]
        crop_col = col - self.crop_origin[1]
        return crop_row, crop_col

    def plot_path(self, start_latlon, end_latlon):
        start_pose = latLonYaw2Geopose(*start_latlon, 0.0)
        end_pose = latLonYaw2Geopose(*end_latlon, 0.0)

        if self.use_terrain_path_planner:
            path = terrainPathPlanner(
                start_pose,
                end_pose,
                self.waypoint_distance,
                self.elevation_cost,
                self.elevation_limit,
                self.roll_cost,
                self.roll_limit
            )
        else:
            path = basicPathPlanner(start_pose, end_pose, self.waypoint_distance)

        print(f"Generated path with {len(path)} points.")

        # Convert to pixel coordinates
        pixel_coords_absolute = [
            rowcol(self.transform, *utm.from_latlon(p.position.latitude, p.position.longitude)[:2])
            for p in path
        ]

        rows, cols = zip(*pixel_coords_absolute)
        min_row, max_row = max(min(rows) - self.margin, 0), min(max(rows) + self.margin, self.full_image.shape[0])
        min_col, max_col = max(min(cols) - self.margin, 0), min(max(cols) + self.margin, self.full_image.shape[1])

        # Crop image and store origin
        self.cropped_image = self.full_image[min_row:max_row, min_col:max_col]
        self.crop_origin = (min_row, min_col)

        # Recalculate pixel coordinates relative to cropped image
        pixel_coords = [(r - min_row, c - min_col) for r, c in pixel_coords_absolute]

        # Plot
        plt.imshow(self.cropped_image, cmap='terrain')
        y_vals, x_vals = zip(*pixel_coords)
        plt.plot(x_vals, y_vals, color='blue', marker='o', markersize=3, label='Path')
        plt.title("Planned Path on Cropped Terrain")
        plt.legend()
        plt.colorbar(label='Elevation')
        plt.show()


if __name__ == "__main__":
    tiff_path = "autonomy/maps/slate_canyon.tif"
    planner = Planner(tiff_path)

    start = (40.2231137, -111.6273118)  # lat, lon
    end = (40.2233902, -111.6271029)    # lat, lon
    planner.plot_path(start, end)