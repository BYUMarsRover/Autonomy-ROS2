import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from rover_msgs.srv import PlanPath, OrderPath, OrderAutonomyWaypoint
from rover_msgs.msg import AutonomyTaskInfo, RoverStateSingleton
from ublox_read_2.msg import PositionVelocityTime
from nav_msgs.msg import Path
from geometry_msgs.msg import Point, PoseStamped, Pose, Quaternion
from std_msgs.msg import Header, Int8
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

        # Subscribers
        self.create_subscription(RoverStateSingleton, '/odometry/rover_state_singleton', self.rover_state_singleton_callback, 10) #Rover GPS and Heading
        # self.location = (40.3224, -111.6436) # NOTE: gravel pits placeholder
        self.location = (38.406441, -110.791932) # NOTE: hanksville placeholder

        # Services
        # Plans order of waypoints to visit mapviz version and Autonomy Wayponit version
        self.plan_order_service = self.create_service(OrderAutonomyWaypoint, '/plan_order', self.plan_order)
        self.plan_order_mapviz_service = self.create_service(OrderPath, '/plan_order_mapviz', self.plan_order_mapviz)
        # This service plans a path from start to goal using slope as cost
        self.plan_path_service = self.create_service(PlanPath, 'plan_path', self.plan_path)

        # Clients

        # Timer for the loop function
        self.timer = self.create_timer(0.2, self.loop) # Perform the loop function at 5Hz

        # Initialize Mapper object with asc file
        # Gravel Pit Map
        if self.location[0] > 40.3166 and self.location[0] < 40.323302 and self.location[1] > -111.649553 and self.location[1] < -111.6369:
            self.get_logger().info("Welcome to the gravel pit!")
            file_path=os.path.join(get_package_share_directory('path_planning'), 'data', 'gravel_pits.asc')
            self.eMapper = Mapper(file_path=file_path, zone=12, zone_letter='T') # TODO: verify zone and zone_letter
            # self.eMapper.chop_map(200, 700, 0, 500)
        # Hanksville Map
        elif self.location[0] > 38.392509 and self.location[0] < 38.450525 and self.location[1] > -110.804971 and self.location[1] <  -110.773991:
            self.get_logger().info("Welcome to Hanksville!")
            file_path=os.path.join(get_package_share_directory('path_planning'), 'data', 'hanksville_station.asc')
            self.eMapper = Mapper(file_path=file_path, zone=12, zone_letter='S')
        else:
            print("Current location not supported")
            self.get_logger().warn("Current location not supported for path planning")

        # Initialize PathPlanner object with elevation map, elevation weight, and slope degree threshold
        self.path_planner = AStarPlanner(cost_map=self.eMapper.map, ew=30., e_thresh=10)
        self.get_logger().info("Path Planning is ready.")
        self.path_needed = False

        ################# Mapviz Communication Setup #################

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

    def rover_state_singleton_callback(self, msg):
        self.location = (msg.gps.latitude, msg.gps.longitude)
        return

    def loop(self):
        if self.path_needed:
            # Plan path - pass start and goal coordinates in x, y format using the Mapper function
            path, length = self.path_planner.plan_path(self.start, self.goal)

            # Get waypoints along path - see AStar.py for function description
            waypoints = self.path_planner.get_path_waypoints(dist_between_wp=10)

            # Get explored nodes
            explored_nodes = self.path_planner.get_explored_nodes()

            # Visualize path
            visualize_path(path, self.eMapper.grad_map, waypoints=waypoints, explored_nodes=explored_nodes)

            self.path_needed = False

            # Publish waypoints along path to /mapviz/path topic
            # path_msg = Path()
            # path_msg.header.frame_id = 'map'
            # path_msg.header.stamp = self.get_clock().now().to_msg()
            # for n in waypoints:
            #     lat, lon = self.eMapper.xy_to_latlon(n[1], n[0])
            #     utm_coords = utm.from_latlon(lat, lon)
            #     x = utm_coords[0] - self.utm_easting_zero
            #     y = utm_coords[1] - self.utm_northing_zero
            #     path_msg.poses.append(
            #         PoseStamped(
            #             header=Header(frame_id='map'),
            #             pose=Pose(
            #                 position=Point(x=x, y=y, z=0.0),
            #                 orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            #             )
            #         )
            #     )

            # self.mapviz_path.publish(path_msg) #TODO: verify that the path pops up on mapviz **Current Task


    # Plan Waypoint Order Service Callback
    def plan_order(self, request, response):
        '''
        Plan the order of waypoints to visit
            Pass in: List of AutonomyTaskInfo messages with poses in lat/lon format
            Returns: List of AutonomyTaskInfo messages with waypoints reordered
            NOTE: will take too long for more than 8 waypoints
        '''
        self.get_logger().info('Planning Order')
        in_ids_order = request.ids
        in_order = request.task_list
        if len(in_order) > 8:
            response.success = False #TODO: verify that "success" can be set to False
            self.get_logger().info("Too many waypoints") #TODO: get this message somewhere useful
            return response
        
        wp = []
        for n in in_order:
            wp.append(self.eMapper.latlon_to_xy(n.latitude, n.longitude))
        start = [self.eMapper.latlon_to_xy(self.location[0], self.location[1])]


        _, ids = self.path_planner.plan_wp_order(start, wp)

        for n in ids:

            # Grab the original lat lon coords in order they were planned
            lat = in_order[n].latitude
            lon = in_order[n].longitude
            id = in_order[n].tag_id

            # Append the waypoints to the response in the order they should be visited
            response.ids.append(Int8(data=in_ids_order[n].data))
            response.task_list.append(AutonomyTaskInfo(latitude=lat, longitude=lon, tag_id=id))

        response.success = True
        response.message = str("Waypoints order planned")
        return response

    # Plan Waypoint Order Service Callback for mapviz
    def plan_order_mapviz(self, request, response):
        '''
        Plan the order of waypoints to visit
            Pass in: Path message with poses in lat/lon format
            Returns: Path message with waypoints reordered
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
        start = (request.start.x, request.start.y)
        goal = (request.goal.x, request.goal.y)
        self.start = self.eMapper.latlon_to_xy(start[0], start[1])
        self.goal = self.eMapper.latlon_to_xy(goal[0], goal[1])
        self.path_needed = True

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