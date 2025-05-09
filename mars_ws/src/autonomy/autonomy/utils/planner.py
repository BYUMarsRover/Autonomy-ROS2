# Planner
from autonomy.utils.gps_utils import (
    latLonYaw2Geopose,
    meters2LatLon,
    latLon2Meters,
)
from autonomy.utils.plan_utils import (
    basicPathPlanner,  # plan a straight line between two GPS coordinates
    basicOrderPlanner,  # use brute force to find the best order of legs (based on distance)
)
from autonomy.utils.terrain_utils import (
    terrainPathPlanner,
    terrainOrderPlanner,
)
import utm
import math
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from collections import deque

## Origin getting TODO fix later
import os
import yaml
from ament_index_python.packages import get_package_share_directory
import numpy as np

class Planner():
    def __init__(self, node):
        self.node = node
        
        # Waypoint path parameters
        node.declare_parameter("use_terrain_path_planner", True)
        node.declare_parameter("use_terrain_order_planner", False)
        node.declare_parameter("elevation_cost", 0.1)
        node.declare_parameter("waypoint_distance", 15.0)
        node.declare_parameter("elevation_limit", 0.7)
        node.declare_parameter("roll_cost", 1.0)
        node.declare_parameter("roll_limit", 1.4)

        # Search Path Parameters
        # TAKE INTO ACCOUNT THE FACT THERE IS A THRESHOLD ON COMPLETING ON OF THE WAYPOINTS
        node.declare_parameter("spiral_inital_radius", 4.0)
        node.declare_parameter("spiral_growth_factor", 1.0) # Meter in radius growth beteween each point
        node.declare_parameter("spiral_step_size", 45.0) # Spiral Angle step size (degrees) beteween each point

        self.use_terrain_path_planner = node.get_parameter("use_terrain_path_planner").value
        self.use_terrain_order_planner = node.get_parameter("use_terrain_order_planner").value

        self.path_publisher = node.create_publisher(Path, 'waypoint/path', 10)
        self.search_path_publisher = node.create_publisher(Path, 'path/search', 10)

        # TODO FIX THE ORIGIN!!!
        # Find origin
        node.declare_parameter('map', 'hanksville') # Declare Mapviz Location Parameter (passed in from launch file)
        mapviz_location = node.get_parameter('map').value # Extract location
        mapviz_origins_path = os.path.join(get_package_share_directory('mapviz_tf'), 'params', 'mapviz_origins.yaml')
        # Open mapviz origins file
        with open(mapviz_origins_path, 'r') as file:
            mapviz_origins = yaml.safe_load(file)
        for location in mapviz_origins: # iterate over dictionaries to find the lat/lon of the location
            if location['name'] == mapviz_location:
                lat = location['latitude']
                lon = location['longitude']
                node.get_logger().info(f"Mapviz location: {mapviz_location} ({lat}, {lon})")
                break

        utm_origin = utm.from_latlon(lat, lon)

        self.utm_easting_zero = utm_origin[0]
        self.utm_northing_zero = utm_origin[1]

        # UTM zone and hemisphere (will set on first gps fix)
        self.zone = None
        self.hemisphere = None
        self.filtered_gps = None

        self.path = deque() # TODO consider making this a queue?? or deque
        self.search_path = deque() # TODO consider making this a queue?? or deque

        self.use_planned_path = False



    def clear_path(self):
        self.path.clear()
        self.use_planned_path = False

    def navigate_helper(self, start_wp, dest_wp):

        """

        """
        # TODO: Move this somewhere else?
        # Set zone and hemisphere for UTM conversions
        # What is zone and hemisphere used for?
        path = None
        if (self.zone is None) or (self.hemisphere is None):
            _, _, self.zone, self.hemisphere = utm.from_latlon(start_wp.position.latitude, start_wp.position.longitude)


        # TODO do like a try except to make sure something doesnt error out and everything looks good
        # 1. Generate a path to the destination waypoint
        distance = self.node.get_parameter("waypoint_distance").value
        if self.use_terrain_path_planner:
            cost = self.node.get_parameter("elevation_cost").value
            limit = self.node.get_parameter("elevation_limit").value
            roll_cost = self.node.get_parameter("roll_cost").value
            roll_limit = self.node.get_parameter("roll_limit").value
            path = terrainPathPlanner(start_wp, dest_wp, distance, cost, limit, roll_cost, roll_limit)
        
        if path is None:
            path = basicPathPlanner(start_wp, dest_wp, distance)
        
        self.path.clear()
        self.path.extend(path)
        
        # take off the last point because it will be close to the destination
        self.path.pop()


        # TODO if statement if we want to publish the planned path
        # Path to Mapviz
        self.publish_path_map(self.path, self.path_publisher)

        # TODO: Publish the circles around each point

    
    def search_helper(self, center_wp):
        """
        Generates a set of GeoPose waypoints around a center point using local x/y offsets.
        Also publishes the planned path as a nav_msgs/Path message.
        """
        # search_points = [
        #     (4.5, 7.79),
        #     (9.0, 0.0),
        #     (4.5, -7.79),
        #     (-4.5, -7.79),
        #     (-9.0, 0.0),
        #     (-4.5, 7.79),
        #     (0.0, 15.58),
        #     (13.5, 7.79),
        #     (13.5, -7.79),
        #     (0.0, -15.58),
        #     (-13.5, -7.79),
        #     (-13.5, 7.79),
        # ]
        scalar = 1.0
        # Make a bunch of x y points in a spriral outward
        initial_radius = self.node.get_parameter("spiral_inital_radius").value
        growth_factor = self.node.get_parameter("spiral_growth_factor").value
        step_deg = self.node.get_parameter("spiral_step_size").value
        search_points = generate_spiral_points(20, initial_radius, growth_factor, step_deg)
        # Multiply all the points by the scalar factor

        center_lat = center_wp.position.latitude
        center_lon = center_wp.position.longitude

        # Convert center to UTM
        center_easting, center_northing, zone_number, zone_letter = utm.from_latlon(center_lat, center_lon)

        self.search_path.clear()

        for dx, dy in search_points:
            x = center_easting + dx
            y = center_northing + dy

            lat, lon = utm.to_latlon(x, y, zone_number, zone_letter)
            # IGNORE YAW FOR NOW
            # yaw = math.atan2(dx, dy)  # Facing away from center
            yaw = 0.0

            geopose = latLonYaw2Geopose(lat, lon, yaw)
            self.search_path.append(geopose)

        # Optional: publish to Mapviz
        self.publish_path_map(self.search_path, self.search_path_publisher)


    def publish_path_map(self, path, publisher):
        path_msg = Path()
        path_msg.header.stamp = self.node.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        # Append coordinates to the waypoint list in lat/lon format
        for geopose in path:
            pose = PoseStamped()
            utm_coords = utm.from_latlon(geopose.position.latitude, geopose.position.longitude)

            pose.header = path_msg.header
            pose.pose.position.x = utm_coords[0] - self.utm_easting_zero
            pose.pose.position.y = utm_coords[1] - self.utm_northing_zero
            
            path_msg.poses.append(pose)

        publisher.publish(path_msg)


def generate_spiral_points(num_points=20, a=2.0, b=1.0, step_deg=45.0):
    """
    Generate (x, y) points in an outward spiral pattern using polar coordinates.

    Parameters:
    - num_points (int): Number of points to generate along the spiral.
    - a (float): Initial radius offset of the spiral. Controls how far the spiral starts from the center.
    - b (float): Growth factor of the radius. Larger values make the spiral expand more quickly.
    - step_deg (float): Angular step size between points in the spiral.

    Returns:
    - List of (x, y) tuples representing points in the spiral.
    """
    search_points = []
    for i in range(num_points):
        step_rad = math.radians(step_deg)
        theta = i * step_rad  # controls angle step
        r = a + b * i              # radius increases linearly
        x = r * math.cos(theta)
        y = r * math.sin(theta)
        search_points.append((x, y))
    return search_points