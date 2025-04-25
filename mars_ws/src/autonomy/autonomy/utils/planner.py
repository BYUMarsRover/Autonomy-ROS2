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
        
        node.declare_parameter("use_terrain_path_planner", False)
        node.declare_parameter("use_terrain_order_planner", False)
        node.declare_parameter("elevation_cost", 0.1)
        node.declare_parameter("waypoint_distance", 15.0)

        self.use_terrain_path_planner = node.get_parameter("use_terrain_path_planner").value
        self.use_terrain_order_planner = node.get_parameter("use_terrain_order_planner").value
        self.elevation_cost = node.get_parameter("elevation_cost").value
        self.waypoint_distance = node.get_parameter("waypoint_distance").value

        self.path_publisher = node.create_publisher(Path, 'waypoint/path', 10)


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
        if self.use_terrain_path_planner:
            path = terrainPathPlanner(start_wp, dest_wp, self.waypoint_distance, self.elevation_cost)
        else:
            path = basicPathPlanner(start_wp, dest_wp, self.waypoint_distance)
        
        self.path.clear()
        self.path.extend(path)

        # TODO Maybe take off the last point if it includes the destination

        # TODO if statement if we want to publish the planned path
        # Path to Mapviz
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        # Append coordinates to the waypoint list in lat/lon format
        for geopose in self.path:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.position.x = geopose.position.latitude
            pose.position.y = geopose.position.longitude
            
            path_msg.poses.append(pose)

        self.path_publisher.publish(path_msg)

        
        # TODO publish the path instead of the waypoints?
        # for the path message do it in xy

        # # 2. Publish the GPS positions to mapviz
        # for wp in path:
        #     navsat_fix = NavSatFix()
        #     navsat_fix.header.frame_id = "map"
        #     navsat_fix.header.stamp = self.get_clock().now().to_msg()
        #     navsat_fix.latitude = wp.position.latitude
        #     navsat_fix.longitude = wp.position.longitude

        #     # 2.1 Publish to different topics based on type
        #     if wp != dest_wp:
        #         self.mapviz_inter_publisher.publish(navsat_fix)
        #     else:
        #         self.mapviz_goal_publisher.publish(navsat_fix)
        #     time.sleep(0.1)  # give time to publish