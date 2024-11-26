import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from rover_msgs.srv import PlanPath
from rover_msgs.msg import PointList
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
        self.path_pub = self.create_publisher(PointList, 'path', 10)

        # Subscribers
        # TODO: subscription to know the current lat lon position of the rover

        # Services
        self.plan_path_service = self.create_service(PlanPath, 'plan_path', self.plan_path)

        # Clients

        # Check Actions Needed
        self.timer = self.create_timer(0.1, self.action)

        # Initialize Mapper object with ascii file
        file_path=os.path.join(get_package_share_directory('path_planning'), 'data', 'gravel_pits.asc')
        self.eMapper = Mapper(file_path=file_path, zone=12, zone_letter='N')
        self.eMapper.chop_map(200, 700, 0, 500)

        # Initialize PathPlanner object
        self.path_planner = AStarPlanner(cost_map=self.eMapper.map, ew=30., e_thresh=10)
        self.path_needed = False

    def action(self):
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

    def plan_order(self, request, response):
        '''
        Plan the order of waypoints to visit
            Pass in: the waypoints in lat/lon format
            Returns: the optimal order of waypoints in lat/lon format
        '''
        # Convert lat/lon to x/y for order planning
        wp = []
        for n in request.points:
            wp.append(self.eMapper.latlon_to_xy(n.x, n.y))

        order = self.path_planner.plan_order(wp)

        wp = []
        for n in order:
            wp.append(self.eMapper.xy_to_latlon(n[0], n[1]))
        
        for point in wp:
            response.points.append(PointList(x=point[0], y=point[1]))

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