import numpy as np
import os
from ament_index_python.packages import get_package_share_directory

from e_mapping import Mapper

from AStar import *

'''
This script demonstrates how to use the AStarPlanner class and the Mapper class 
to plan the best path through an elevation map.
'''

# File path to the ascii file - this should be an elevation map
file_path=os.path.join(get_package_share_directory('path_planning'), 'data', 'gravel_pits.asc')

def main():

    # Initialize Mapper object with ascii file
    eMapper = Mapper(file_path=file_path, zone=12, zone_letter='N')
    
    # Chop the imported map to the desired size
    eMapper.chop_map(200, 700, 0, 500)

    # Display the Elevation map
    eMapper.display_map(map_type='Elevation')

    # Display the Slope map
    eMapper.display_map(map_type='Slope')

    # Initialize PathPlanner object
    '''
    TODO: Tune ew based on how much you want slopes to "cost" in the path planning algorithm
    ew: weight given to the cost of going up slopes
    e_thresh: (degrees) the path will ignore nodes with slopes greater than this value
    '''
    path_planner = AStarPlanner(cost_map=eMapper.map, ew=30., e_thresh=10)

    # Declare start and goal in x, y coordinates (use mapper if in lat, lon)
    start = eMapper.latlon_to_xy(40.32035949015593, -111.64246891845703)
    goal = eMapper.latlon_to_xy(40.318824735304105, -111.64304286671711)
    # Corresponding x, y coordinates for the above lat, lon values
    # start = (400, 80)
    # goal = (350, 250)

    # Plan path - pass start and goal coordinates in x, y format using the Mapper function
    path, length = path_planner.plan_path(start, goal)
    print(f"Path Length: {length}")

    # Get waypoints along path - see AStar.py for function description
    waypoints = path_planner.get_path_waypoints(dist_between_wp=10)

    # Get explored nodes
    explored_nodes = path_planner.get_explored_nodes()

    # Visualize path
    visualize_path(path, eMapper.grad_map, waypoints=waypoints, explored_nodes=explored_nodes)

if __name__ == "__main__":
    main()