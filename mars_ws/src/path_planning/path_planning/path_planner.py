import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from rover_msgs.srv import PlanPath, OrderPath
from rover_msgs.msg import RoverStateSingleton, PointList
from ublox_read_2.msg import PositionVelocityTime
from nav_msgs.msg import Path
from geometry_msgs.msg import Point, PoseStamped, Pose, Quaternion
from std_msgs.msg import Header
import os
import yaml
import utm

from .e_mapping import Mapper

from .AStar import *

'''
Created by: Daniel Webb
Date: 11/24/2024

Path Planner Node for managing the interface between the path planning algorithm and the state machina and mapviz
    Features:
        - Uses the AStar object to plan the best path between two points based on a cost map and distance
        - Uses the Mapper object to store a local elevation map and convert between lat/lon and x/y coordinates
        - Uses the AStar object to plan the order of waypoints to visit based on total path length TODO: modify the AStar object to take into account elevation change between waypoints

'''

class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')
        self.get_logger().info("Path Planner Node Started")

        # Publishers
        self.mapviz_path = self.create_publisher(Path, '/mapviz/path', 10)
        self.waypoint_pub = self.create_publisher(PointList, '/navigation/waypoints', 10)

        # Subscribers
        self.create_subscription(RoverStateSingleton, '/odometry/rover_state_singleton', self.rover_state_singleton_callback, 10)   # NOTE: self.location is defined in rover_state_singleton_callback
        # self.location = (40.3224, -111.6436) # NOTE: gravel pits placeholder
        self.create_subscription(PositionVelocityTime, '/rover/PosVelTime', self.update_location, 10) #GPS info from rover
        # TODO: subscription to know the current lat lon position of the rover
        self.location = None        #initialize location
        # self.location = (40.3224, -111.6436) # NOTE: gravel pits placeholder    
        # self.location = (38.4231, -110.7851) # NOTE: hanksville placeholder

        # Services
        # This service plans the order of waypoints to visit
        # self.plan_order_service = self.create_service(PointList, 'plan_order', self.plan_order) #TODO: I don't believe that this has been developed
        self.plan_order_service = self.create_service(OrderPath, 'plan_order', self.plan_order)
        # This service plans a path from start to goal using slope as cost
        self.plan_path_service = self.create_service(PlanPath, 'plan_path', self.plan_path) #TODO: implement path planning points into state machine

        # Clients

        # Timer for the loop function
        self.timer = self.create_timer(0.2, self.loop) # Perform the loop function at 5Hz

             ################ Mapviz Communication Setup ################# #TODO: Look into this please!

        # Retrieve Mapviz Location
        self.declare_parameter('location', 'gravel_pit')
        location = self.get_parameter('location').value

        # Use Location to get the lat and lon corresponding to the mapviz (0, 0) coordinate
        mapviz_params_path = os.path.join(get_package_share_directory('mapviz_tf'), 'params', 'mapviz_params.yaml')
        lat, lon = get_coordinates(mapviz_params_path, location)

        # Convert lat/lon to UTM coordinates
        utm_coords = utm.from_latlon(lat, lon)
        self.utm_easting_zero = utm_coords[0]
        self.utm_northing_zero = utm_coords[1]
        self.utm_zone_number = utm_coords[2]
        self.utm_zone_letter = utm_coords[3]

        # # Initialize Mapper object with asc file
        # # Gravel Pit Map
        # if self.location[0] > 40.3166 and self.location[0] < 40.323302 and self.location[1] > -111.649553 and self.location[1] < -111.6369:
        #     self.get_logger().info("Welcome to the gravel pit! Path Planning is ready.")
        #     file_path=os.path.join(get_package_share_directory('path_planning'), 'data', 'gravel_pits.asc')
        #     self.eMapper = Mapper(file_path=file_path, zone=12, zone_letter='T') # TODO: verify zone and zone_letter
        #     # self.eMapper.chop_map(200, 700, 0, 500)
        # # Hanksville Map
        # elif self.location[0] > 38.392509 and self.location[0] < 38.450525 and self.location[1] > -110.804971 and self.location[1] <  -110.773991:
        #     self.get_logger().info("Welcome to Hanksville! Path Planning is ready.")
        #     file_path=os.path.join(get_package_share_directory('path_planning'), 'data', 'hanksville_full.asc')
        #     self.eMapper = Mapper(file_path=file_path, zone=12, zone_letter='S')
        # else:
        #     print("Current location not supported")
        #     self.get_logger().warn("Current location not supported for path planning")

        self.initialize_mapper()

    def initialize_mapper(self):
        # Ensure location is set before using it
        if self.location is None:
            self.get_logger().warn("Location is not yet available. Deferring mapper initialization.")
            self.path_needed = False
            return

        # Check location and initialize the appropriate mapper
        if 40.3166 < self.location[0] < 40.323302 and -111.649553 < self.location[1] < -111.6369:
            self.get_logger().info("Welcome to the gravel pit! Path Planning is ready.")
            file_path = os.path.join(get_package_share_directory('path_planning'), 'data', 'gravel_pits.asc')
            self.eMapper = Mapper(file_path=file_path, zone=12, zone_letter='T')
        elif 38.392509 < self.location[0] < 38.450525 and -110.804971 < self.location[1] < -110.773991:
            self.get_logger().info("Welcome to Hanksville! Path Planning is ready.")
            file_path = os.path.join(get_package_share_directory('path_planning'), 'data', 'hanksville_full.asc')
            self.eMapper = Mapper(file_path=file_path, zone=12, zone_letter='S')
        else:
            self.get_logger().warn("Current location not supported for path planning.")
            return
        
                # Initialize PathPlanner object with elevation map, elevation weight, and slope degree threshold
        self.path_planner = AStarPlanner(cost_map=self.eMapper.map, ew=30., e_thresh=10)
        self.path_needed = False

    def check_location(self, goal):
        # Check location
        if 40.3166 < goal[0] < 40.323302 and -111.649553 < goal[1] < -111.6369:
            self.get_logger().info("Gravel Pits destination set")
        elif 38.392509 < goal[0] < 38.450525 and -110.804971 < goal[1] < -110.773991:
            self.get_logger().info("Hanksville destination set")
        else:
            self.get_logger().warn("Current location not supported for path planning.")
            return
        self.goal = self.eMapper.latlon_to_xy(goal[0], goal[1])
        self.get_logger().info("Initializing path planner")
        self.path_needed = True


    def update_location(self, msg):
        self.location = (msg.lla[0], msg.lla[1])
        # self.location = (msg.latitude, msg.longitude)
        

    def rover_state_singleton_callback(self, msg: RoverStateSingleton):
        self.curr_latitude = msg.gps.latitude
        self.curr_longitude = msg.gps.longitude
        if self.location is None:
            self.location = (self.curr_latitude, self.curr_longitude)
            self.initialize_mapper()
        self.location = (self.curr_latitude, self.curr_longitude)
        


    def loop(self):
        if self.path_needed:
            # Plan path - pass start and goal coordinates in x, y format using the Mapper function
            path, length = self.path_planner.plan_path(self.start, self.goal)
            self.get_logger().info("Path planner completed")

            waypoints = self.path_planner.get_path_waypoints(dist_between_wp=10)
            self.get_logger().info("Waypoints gathered!")

            # Get waypoints along path - see AStar.py for function description
            points = []
            for waypoint in waypoints:
                x, y = waypoint  # Unpack (x, y) tuple
                latlon = self.eMapper.xy_to_latlon(x, y)  # Convert xy to lat/lon
                point = Point()
                point.x = float(latlon[0])  # Latitude
                point.y = float(latlon[1])  # Longitude
                points.append(point)

            # Create a PointList message
            point_list_msg = PointList()
            point_list_msg.points = points
            point_list_msg.tag_id = str(self.goal_tag_id)
            # self.get_logger().info(f"points {points}")
            # self.get_logger().info(f"point_list_msg {point_list_msg}")
            # yaml_message = yaml.dump({"points": [{"x": p.x, "y": p.y, "z": p.z} for p in points]})
            # self.get_logger().info(f"YAML-formatted point_list_msg:\n{yaml_message}")

            # Publish the PointList
            self.waypoint_pub.publish(point_list_msg)
            # self.get_logger().info(f"Publishing waypoints: {points}")

            # Get explored nodes
            explored_nodes = self.path_planner.get_explored_nodes()

            # Visualize path
            visualize_path(path, self.eMapper.grad_map, waypoints=waypoints, explored_nodes=explored_nodes)

            self.path_needed = False

            # Publish waypoints along path to /mapviz/path topic
            path_msg = Path()
            path_msg.header.frame_id = 'map'
            path_msg.header.stamp = self.get_clock().now().to_msg()
            for n in waypoints:
                lat, lon = self.eMapper.xy_to_latlon(n[1], n[0])
                utm_coords = utm.from_latlon(lat, lon)
                x = utm_coords[0] - self.utm_easting_zero
                y = utm_coords[1] - self.utm_northing_zero
                path_msg.poses.append(
                    PoseStamped(
                        header=Header(frame_id='map'),
                        pose=Pose(
                            position=Point(x=x, y=y, z=0.0),
                            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                        )
                    )
                )

            self.mapviz_path.publish(path_msg) #TODO: verify that the path pops up on mapviz **Current Task


    # Plan Waypoint Order Service Callback
    def plan_order(self, request, response):
        '''
        Plan the order of waypoints to visit
            Pass in: the waypoints in lat/lon format
            Returns: the optimal order of waypoints in lat/lon format
            NOTE: will take too long for more than 8 waypoints
        '''
        in_path = request.path

        if len(in_path.poses) > 8:
            response.success = False #TODO: verify that "success" can be set to False
            self.get_logger().info("Too many waypoints") #TODO: get this message somewhere useful
            return response
        
        wp = []
        # Convert lat/lon to x/y for order planning
        for n in in_path.poses:
            wp.append(self.eMapper.latlon_to_xy(n.pose.position.x, n.pose.position.y))
        start = [self.eMapper.latlon_to_xy(self.location[0], self.location[1])]

        _, ids = self.path_planner.plan_wp_order(start, wp)

        for n in ids:
            # Grab the original lat lon coords in order they were planned
            lat = in_path.poses[n].pose.position.x
            lon = in_path.poses[n].pose.position.y

            # Append the waypoints to the response in the 
            # order they should be visited
            response.path.poses.append(
                PoseStamped(
                    header=in_path.poses[n].header, 
                    pose=Pose(
                        position=Point(x=lat, y=lon, z=0.0),
                        orientation=in_path.poses[n].pose.orientation
                    )
                )
            )
        response.path.header = in_path.header
        response.success = True
        return response

    # Plan Path Service Callback
    def plan_path(self, request, response):
        '''
        This Service sets a flag for the loop function 
        to plan a path which will get published to the 
        "path" topic when finished

        Pass in: start and goal in lat/lon format
        '''
        # Convert lat/lon to x/y for path planning
        # start = (request.start.x, request.start.y)
        goal = (request.goal.x, request.goal.y)
        self.goal_tag_id = request.tag_id 
        # self.start = self.eMapper.latlon_to_xy(start[0], start[1])
        self.start = self.eMapper.latlon_to_xy(self.curr_latitude, self.curr_longitude)
        self.check_location(goal)

        response.received = True
        return response
    
def get_coordinates(file_path, location):
    # Read the YAML file
    with open(file_path, 'r') as file:
        data = yaml.safe_load(file)
    
    # Navigate to the locations data
    locations = data['/**']['ros__parameters']['locations']
    
    # Check if the location exists
    if location in locations:
        lat = locations[location]['latitude']
        lon = locations[location]['longitude']
        return lat, lon
    else:
        return None
    
def main(args=None):

    rclpy.init(args=args)

    path_planner = PathPlanner()

    rclpy.spin(path_planner)

    path_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()