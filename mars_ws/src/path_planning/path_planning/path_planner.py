import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from rover_msgs.srv import PlanPath, PointList
import os

from .e_mapping import Mapper

from .AStar import *

# TODO: make parameter file for importing different maps
# map = os.path.join(get_package_share_directory('path_planning'), 'data', 'gravel_pits.asc')

class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')
        self.get_logger().info("Path Planner Node Started")

        # Publishers
        # TODO: path publisher

        # Subscribers
        # TODO: subscription to know the current lat lon position of the rover
        self.location = (40.3224, -111.6436) # NOTE: gravel pits placeholder

        # Services
        # This service plans the order of waypoints to visit
        self.plan_order_service = self.create_service(PointList, 'plan_order', self.plan_order)
        # This service plans a path from start to goal using slope as cost
        self.plan_path_service = self.create_service(PlanPath, 'plan_path', self.plan_path)

        # Clients

        # Check Actions Needed
        self.timer = self.create_timer(0.1, self.loop)

        # Initialize Mapper object with ascii file
        if self.location[0] > -111.649553 and self.location[1] < -111.6369 and self.location[0] > 40.3166 and self.location[1] < 40.323302:
            self.get_logger().info("Welcome to the gravel pits! Path Planning is ready.")
            file_path=os.path.join(get_package_share_directory('path_planning'), 'data', 'gravel_pits.asc')
            self.eMapper = Mapper(file_path=file_path, zone=12, zone_letter='N')
            self.eMapper.chop_map(200, 700, 0, 500)
        else:
            print("Current location not supported")
            self.get_logger().warn("Current location not supported for path planning")

        # Initialize PathPlanner object
        self.path_planner = AStarPlanner(cost_map=self.eMapper.map, ew=30., e_thresh=10)
        self.path_needed = False

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

    # Plan Waypoint Order Service Callback
    def plan_order(self, request, response):
        '''
        Plan the order of waypoints to visit
            Pass in: the waypoints in lat/lon format
            Returns: the optimal order of waypoints in lat/lon format
            NOTE: will take too long for more than 8 waypoints
        '''
        if len(request.points) > 8:
            response.success = False
            self.get_logger().info("Too many waypoints") #TODO: get this message somewhere useful
            return response
        
        wp = []
        # Convert lat/lon to x/y for order planning
        for n in request.points:
            wp.append(self.eMapper.latlon_to_xy(n.x, n.y))
        start = self.eMapper.latlon_to_xy(self.location)

        _, ids = self.path_planner.plan_order(start, wp)

        for n in ids:
            # Grab the original lat lon coords in order they were planned
            lat = request.points[n].x
            lon = request.points[n].y

            # Append the waypoints to the response in the 
            # order they should be visited
            response.points.append(PointList(x=lat, y=lon))

        return response

    # Plan Path Service Callback
    def plan_path(self, request, response):
        '''
        This Service sets a flag to plan a path which will get
        published to the "path" topic when finished

        Pass in: start and goal in lat/lon format
        '''
        # Convert lat/lon to x/y for path planning
        start = (request.start.x, request.start.y)
        goal = (request.goal.x, request.goal.y)
        self.start = self.eMapper.latlon_to_xy(start)
        self.goal = self.eMapper.latlon_to_xy(goal)
        self.path_needed = True

        response.success = True
        return response
    
def main(args=None):

    rclpy.init(args=args)

    path_planner = PathPlanner()

    rclpy.spin(path_planner)

    path_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()