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
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from collections import deque

class Planner():
    def __init__(self, node):
        self.node = node
        
        node.declare_parameter("use_terrain_path_planner", True)
        node.declare_parameter("use_terrain_order_planner", False)
        node.declare_parameter("elevation_cost", 0.1)
        node.declare_parameter("waypoint_distance", 15.0)

        self.use_terrain_path_planner = node.get_parameter("use_terrain_path_planner").value
        self.use_terrain_order_planner = node.get_parameter("use_terrain_order_planner").value

        self.path_publisher = node.create_publisher(Path, 'waypoint/path', 10)

        utm_origin = utm.from_latlon(40.2497218, -111.649276)
        self.utm_easting_zero = utm_origin[0]
        self.utm_northing_zero = utm_origin[1]

        self.search_points = [
            (4.5, 7.79),
            (9.0, 0.0),
            (4.5, -7.79),
            (-4.5, -7.79),
            (-9.0, 0.0),
            (-4.5, 7.79),
            (0.0, 15.58),
            (13.5, 7.79),
            (13.5, -7.79),
            (0.0, -15.58),
            (-13.5, -7.79),
            (-13.5, 7.79),
        ]

        # UTM zone and hemisphere (will set on first gps fix)
        self.zone = None
        self.hemisphere = None
        self.filtered_gps = None

        self.path = deque() # TODO consider making this a queue?? or deque

        self.use_planned_path = False


    def clear_path(self):
        self.path.clear()
        self.use_planned_path = False

    def navigate_helper(self, start_wp, dest_wp, hex_mode=False, found_mode=False):

        """

        """
        # TODO: Move this somewhere else?
        # Set zone and hemisphere for UTM conversions
        # What is zone and hemisphere used for?
        if (self.zone is None) or (self.hemisphere is None):
            _, _, self.zone, self.hemisphere = utm.from_latlon(start_wp.position.latitude, start_wp.position.longitude)


        # TODO do like a try except to make sure something doesnt error out and everything looks good
        # 1. Generate a path to the destination waypoint
        distance = self.node.get_parameter("waypoint_distance").value
        if self.use_terrain_path_planner:
            cost = self.node.get_parameter("elevation_cost").value
            path = terrainPathPlanner(start_wp, dest_wp, distance, cost)
        else:
            path = basicPathPlanner(start_wp, dest_wp, distance)
        
        self.path.clear()
        self.path.extend(path)

        # TODO Maybe take off the last point if it includes the destination

        # TODO if statement if we want to publish the planned path
        # Path to Mapviz
        path_msg = Path()
        path_msg.header.stamp = self.node.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        # Append coordinates to the waypoint list in lat/lon format
        for geopose in self.path:
            pose = PoseStamped()
            utm_coords = utm.from_latlon(geopose.position.latitude, geopose.position.longitude)

            pose.header = path_msg.header
            pose.pose.position.x = utm_coords[0] - self.utm_easting_zero
            pose.pose.position.y = utm_coords[1] - self.utm_northing_zero
            
            path_msg.poses.append(pose)

        self.path_publisher.publish(path_msg)

        # TODO: Publish the circles around each point