import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from rover_msgs.srv import PlanPath, OrderPath, OrderAutonomyWaypoint, AutonomyWaypoint
from rover_msgs.msg import AutonomyTaskInfo, RoverStateSingleton
from nav_msgs.msg import Path
from geometry_msgs.msg import Point, PoseStamped, Pose
from std_msgs.msg import Header, Int8
from std_srvs.srv import SetBool 
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

        # Initialize waypoints message that will hold the planned path in lat/lon format ready to send ot the state machine
        self.waypoints = None

        # Initialize path needed flag
        self.path_needed = False

        # Publishers
        self.path_plan_response_pub = self.create_publisher(PlanPath.Response, '/path_plan_response', 10) # for notifying the gui the success status of an attempted path plan
        self.mapviz_path = self.create_publisher(Path, '/mapviz/path', 10)

        # Subscribers
        self.create_subscription(RoverStateSingleton, '/odometry/rover_state_singleton', self.rover_state_singleton_callback, 10)
        self.location = None # initialize location
        # self.location = (40.3224, -111.6436) # NOTE: gravel pits placeholder TODO: remove
        # self.location = (38.4231, -110.7851) # NOTE: hanksville placeholder TODO: remove

        # Services
        # Plans order of waypoints to visit mapviz version and Autonomy Waypoint version
        self.plan_order_service = self.create_service(OrderAutonomyWaypoint, '/plan_order', self.plan_order)
        self.plan_order_mapviz_service = self.create_service(OrderPath, '/plan_order_mapviz', self.plan_order_mapviz) #TODO: Depreciate this
        # This service plans a path from start to goal using slope as cost
        self.plan_path_service = self.create_service(PlanPath, '/plan_path', self.plan_path)
        # This service triggers the AU_waypoint_service sending the path to the state machine when "send waypoints" is clicked in Autonomy GUI
        self.send_path_service = self.create_service(SetBool, '/send_path_service', self.send_path)

        # Clients
        # This client is used to add the self.waypoints to the state machine
        self.send_path_client = self.create_client(AutonomyWaypoint, '/AU_waypoint_service')

        # Timer for the loop function
        self.timer = self.create_timer(0.2, self.loop) # Perform the loop function at 5Hz TODO: tune this

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

        # self.initialize_mapper() # NOTE: not to be done until the rover is publishing state information


    def initialize_mapper(self):
        # Ensure location is set before using it
        if self.location is None:
            self.get_logger().warn("Location is not yet available. Deferring mapper initialization.")
            return

        # Check rover location and initialize using the appropriate elevation map

        # Gravel Pit
        if 40.3166 < self.location[0] < 40.323302 and -111.649553 < self.location[1] < -111.6369:
            self.get_logger().info("Welcome to the gravel pit!")
            file_path = os.path.join(get_package_share_directory('path_planning'), 'data', 'gravel_pits.asc')
            self.eMapper = Mapper(file_path=file_path, zone=12, zone_letter='T')

        # Hanksville Station
        elif 38.402 < self.location[0] < 38.409634 and -110.7962 < self.location[1] < -110.786099:
            self.get_logger().info("Welcome to the Hanksville Station!")
            file_path = os.path.join(get_package_share_directory('path_planning'), 'data', 'hanksville_station.asc')
            self.eMapper = Mapper(file_path=file_path, zone=12, zone_letter='S')

        # 2024 Autonomy Competition Site
        elif 38.422766 < self.location[0] < 38.427038 and -110.787057 < self.location[1] < -110.782198:
            self.get_logger().info("Welcome to the 2024 Autonomy Competition Site!")
            file_path = os.path.join(get_package_share_directory('path_planning'), 'data', '2024_autonomy_competition_site.asc')
            self.eMapper = Mapper(file_path=file_path, zone=12, zone_letter='S')

        # Error Handling for unsupported locations
        else:
            self.get_logger().warn("Current location not supported for path planning.")
            return
        
        # Initialize PathPlanner object with elevation map, elevation weight, and slope threshold (degrees)
        self.path_planner = AStarPlanner(cost_map=self.eMapper.map, ew=30., e_thresh=10)
        self.get_logger().info('Path Planning is ready!')
        self.path_needed = False

    def rover_state_singleton_callback(self, msg: RoverStateSingleton):
        self.curr_latitude = msg.gps.latitude
        self.curr_longitude = msg.gps.longitude
        if self.location is None:
            self.location = (self.curr_latitude, self.curr_longitude)
            self.initialize_mapper()
        self.location = (self.curr_latitude, self.curr_longitude)
        return


    def loop(self):
        if self.path_needed:
            # Initialize waypoint list in lat/lon format ready to be sent to state machine
            self.waypoints = AutonomyWaypoint.Request()

            # Plan path
            path_yx, length = self.path_planner.plan_path(self.start, self.goal)
            self.get_logger().info("Path planned! Length: " + str(length) + " m")

            # Get waypoints every 10 meters along path in x/y format
            waypoints_yx = self.path_planner.get_path_waypoints(dist_between_wp=10) #TODO: tune this distance between points

            # Append coordinates to the waypoint list in lat/lon format
            for i, yx in enumerate(waypoints_yx):
                y, x = yx  # Unpack (y, x) tuple
                lat, lon = self.eMapper.xy_to_latlon(x, y)  # Convert xy to lat/lon

                # Code to print distance and heading between waypoints for field testing
                # if i > 0:
                #     y0 = waypoints_yx[i - 1][0]
                #     x0 = waypoints_yx[i - 1][1]
                #     y1 = waypoints_yx[i][0]
                #     x1 = waypoints_yx[i][1]
                #     dx = x1 - x0
                #     dy = -(y1 - y0)
                #     theta = np.arctan2(dx, dy)
                #     if theta < 0:
                #         theta += 2*np.pi
                #     dist = np.sqrt(dx**2 + dy**2)
                #     self.get_logger().info(f'Dist/Heading: {dist} m/{theta*180/np.pi} deg')
                # self.get_logger().info(f'Waypoint {i}: ({lat}, {lon})')

                self.waypoints.task_list.append(AutonomyTaskInfo(latitude=float(lat), longitude=float(lon), tag_id=self.tag_id))

            # Get explored nodes
            explored_nodes = self.path_planner.get_explored_nodes()

            # Visualize path
            visualize_path(path_yx, self.eMapper.grad_map, waypoints=waypoints_yx, explored_nodes=explored_nodes)

            # Notify the gui that the path is planned
            response = PlanPath.Response()
            response.success = True
            response.message = "Path planned"
            self.path_plan_response_pub.publish(response)

            self.path_needed = False


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
        This Service sets a flag for the loop function to plan a path 
        that is stored in self.waypoints in lat/lon format until 
        requested to be sent to the state machine

        Pass in: goal in lat/lon format
        '''

        self.get_logger().info('Plan Path command received')
        # Convert lat/lon to x/y for path planning
        self.goal = self.eMapper.latlon_to_xy(request.goal.latitude, request.goal.longitude)
        self.start = self.eMapper.latlon_to_xy(self.curr_latitude, self.curr_longitude)

        # Check if the goal coordinate is within the elevation map
        response.success = False
        if self.goal[0] is None and self.start[0] is None:
            response.message = 'Rover & Coordinate outside elevation map, proceed without path planning'
        elif self.goal[0] is None:
            response.message = 'Coordinate outside elevation map, proceed without path planning'
        elif self.start[0] is None:
            response.message = 'Rover outside elevation map, proceed without path planning'
        else:
            self.tag_id = request.goal.tag_id
            self.path_needed = True # Set flag to plan path in loop function
            # Indicate that the goal has been received and the path is being planned
            response.success = True
            response.message = 'Received, planning path...'
        
        return response
    
    # Sends the path to the state machine
    def send_path(self, request, response):
        # Error Handling
        if not self.send_path_client.wait_for_service(timeout_sec=1.0):
            response.success = False
            response.message = "/AU_waypoint_service not available"
            return response
        
        if self.waypoints is not None:
            response.success = True
            response.message = "Sending Waypoint (path included)"
            future = self.send_path_client.call_async(self.waypoints)
            # TODO: add a done callback
        else:
            response.success = False  # Indicate failure
            response.message = "No waypoints available to publish"
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