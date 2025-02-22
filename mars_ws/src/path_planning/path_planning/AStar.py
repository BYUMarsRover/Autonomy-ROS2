import numpy as np
import matplotlib.pyplot as plt
import matplotlib.lines as mlines
import heapq
import itertools

'''
Created by: Daniel Webb
Date: 11/24/2024

AStarPlanner class for path planning using the A* algorithm
    Features:
        - Plan the best path between two points based on a cost map and distance
        - Plan intermediate waypoints along the path based on sharp curves
        - Plan the order of waypoints to visit based on total path length TODO: take into account elevation change between waypoints

    See pathplan_ex.py for an example of how to use the AStarPlanner class
'''

class AStarPlanner:
    def __init__(self, cost_map=None, gw=1., hw=1., ew=1., e_thresh=np.inf, animate=False):

        self.cost_map = cost_map # Expects and elevation map

        # Initialize path list - will contain tuples of (x, y) coordinates for each waypoint
        self.path = []

        # Initialize an array of open nodes that is the size of the entire cost map
        self.nodes = np.empty((cost_map.shape[0], cost_map.shape[1]), dtype=AStarNode)

        # Fill open_nodes with AStarNode objects
        for i in range(cost_map.shape[1]): # x
            for j in range(cost_map.shape[0]): # y
                self.nodes[j, i] = AStarNode(i, j, gw, hw, ew)
                # Precompute neighbors for each node
                self.nodes[j, i].neighbors = self.get_neighbors(self.nodes[j, i])

        self.gw = gw
        self.hw = hw
        self.ew = ew
        # degree slope to m/m
        if e_thresh != np.inf:
            self.e_thresh = np.tan(e_thresh*np.pi/180.)
        else:
            self.e_thresh = np.inf

        self.goal = None
        self.start = None

        self.animate = animate
    
    def plan_path(self, start, goal, count=False):
        '''
        Plans the best path between the start and goal points taking into acount...
            1. path length
            2. cost from the cost map

        Pass in: start and goal points in (x, y) format
        Returns: a list of waypoints in (x, y) format and the path length
        '''

        # Set start and goal nodes
        self.goal = (goal[::-1])
        self.start = (start[::-1])
        self.nodes[self.start].start = True
        self.nodes[self.goal].goal = True

        # Set all nodes to open
        for i in range(self.cost_map.shape[1]): # x
            for j in range(self.cost_map.shape[0]): # y
                self.nodes[j, i].open = True

        # Initialize edge node list as empty
        edge_nodes = [] # list of all nodes to potentially explore

        curr_node = self.start
        i = 0

        # While the goal has not been reached
        while curr_node != self.goal:
            self.nodes[curr_node].open = False

            # Costs of neighbors of current node
            for n in self.nodes[curr_node].neighbors:
                if not self.nodes[n].open: # if neighbor node has already been explored
                    continue # skip neighbor node

                g_step = np.sqrt((curr_node[0] - n[0])**2. + (curr_node[1] - n[1])**2.)
                e_step = np.abs(self.cost_map[n] - self.cost_map[curr_node])/g_step

                if e_step > self.e_thresh: # if elevation change is too great
                    continue # skip the node

                # prioritize diagonal movement
                # if g_step > 1:
                #     g_step -= 0.1
                
                g = self.nodes[curr_node].g + g_step
                e = self.nodes[curr_node].e + np.abs(self.cost_map[n] - self.cost_map[curr_node])

                # if neighbor node is in edge nodes update check to see if the parent node should update
                # the parent node of the neighbor node should only update if the cost to travel 
                # to it from the current node is less than the cost to travel to it from its
                # current parent (its current g and h cost)
                if any(node[1] == n for node in edge_nodes):
                    if self.nodes[n].g*self.gw + self.nodes[n].e*self.ew > g*self.gw + e*self.ew:
                        self.nodes[n].g = g # set g cost of node
                        self.nodes[n].e = e # set e cost of node
                        self.nodes[n].parent = curr_node # reassign the parent of the node
                        self.nodes[n].calc_f()
                        for index, (f_cost, node) in enumerate(edge_nodes):
                            if node == n:
                                edge_nodes[index] = (self.nodes[n].f, n)
                                heapq._siftup(edge_nodes, index)
                                heapq._siftdown(edge_nodes, 0, index)
                                break
                    continue

                # Set parent, h, g, e, and f costs of neighbor node
                self.nodes[n].parent = curr_node # set parent of neighbor node
                self.nodes[n].h = self.get_heuristic_cost(self.nodes[n]) # set h cost of node
                self.nodes[n].g = g # set g cost of node
                self.nodes[n].e = e # set e cost of node
                self.nodes[n].calc_f()

                # Add neighbor node to edge nodes in order of f cost
                heapq.heappush(edge_nodes, (self.nodes[n].f, n))

            # Debugging
            if count:
                i += 1
                print(i)

            curr_node = heapq.heappop(edge_nodes)[1] # set current node to node with lowest f cost

        # Reconstruct path
        path_length = self.nodes[self.goal].g
        while curr_node != self.start:
            self.path.append(curr_node)
            curr_node = self.nodes[curr_node].parent
        self.path.append(self.start)
        self.path.reverse()

        return self.path, path_length

    def get_heuristic_cost(self, node, heuristic='manhattan'):
        '''
        Returns the distance between the node passed and the goal node
            NOTE: manhattan distance by default
        '''
        if heuristic == 'manhattan':
            return abs(node.x - self.goal[1]) + abs(node.y - self.goal[0])
        if heuristic == 'euclidean':
            return np.sqrt((node.x - self.goal[1])**2 + (node.y - self.goal[0])**2)
    
    # Returns a list of all 8 neighbors of a given node
    def get_neighbors(self, node):
        '''
        Returns a list of all 8 neighbors of a given node
            NOTE: less than 8 neightbors if the node is on the edge of the map
        '''
        y_max = self.cost_map.shape[0]
        x_max = self.cost_map.shape[1]
        neighbors = []

        for i in range(-1, 2): # -1, 0, 1
            for j in range(-1, 2): # -1, 0, 1
                if i == 0 and j == 0: # skip current node
                    continue
                # Add indexes of all neighbors that are within the bounds of the cost_map
                if node.x + i >= 0 and node.x + i < x_max and node.y + j >= 0 and node.y + j < y_max:
                    neighbors.append((node.y + j, node.x + i))
        
        return neighbors
    
    def get_path_waypoints(self, dist_between_wp=5):
        '''
        Pass in: desired number of grid units between waypoints
        Returns: a list of waypoints in (x, y) format
            NOTE: This will use the most recent path planned by the planner
            TODO: needs improvement to pick better points
        '''
        waypoints = []
        last_wp = 0
        for i in range(4, len(self.path) - 4):
            p1 = np.array(self.path[i - 4])
            p2 = np.array(self.path[i])
            p3 = np.array(self.path[i + 4])
            
            # Calculate vectors
            v1 = p2 - p1
            v2 = p3 - p2
            
            # Calculate angle between vectors
            angle = np.arccos(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)))
            curvature = np.degrees(angle)
            
            if curvature < 20 and last_wp < i - dist_between_wp:
                waypoints.append(self.path[i])
                last_wp = i

        return waypoints
    
    def get_explored_nodes(self):
        '''
        Returns a list of all nodes explored during the most recent plan_path
        '''
        explored_nodes = []
        for i in range(self.cost_map.shape[1]):
            for j in range(self.cost_map.shape[0]):
                if not self.nodes[j, i].open:
                    explored_nodes.append((j, i))
        return explored_nodes
    
    def plan_wp_order(self, start, wp):
        '''
        Plan the order of waypoints to visit
            Pass in: a list of waypoints in (x, y) format
            Returns: a list of waypoints in optimized order based on distance
        '''
        # Create a set of ids for each wp
        wp_ids = []
        for i in range(len(wp)):
            wp_ids.append(i)

        # All possibilities of orders
        orders_wp = list(itertools.permutations(wp))
        orders_ids = list(itertools.permutations(wp_ids))
        dist = []
        n = len(wp)

        # Calculate the total distance of each order
        for order in orders_wp:
            total_dist = 0
            # Add distance from start to the first waypoint
            total_dist += np.linalg.norm(np.array(start) - np.array(order[0]))
            # Add distance between waypoints
            for i in range(n - 1):
                total_dist += np.linalg.norm(np.array(order[i]) - np.array(order[i + 1]))
            dist.append(total_dist)
            
            # TODO: take into account elevation changes at points between waypoints, 
            # if greater than a certain dxdy threshold, add np.inf to the distance?
            # add up all of the elevation changes that are above a certain threshold?

        return list(orders_wp[np.argmin(dist)]), list(orders_ids[np.argmin(dist)])
        
def visualize_plan_wp_order(start, order):
    '''
    Visualize the order of waypoints to visit
    '''
    order.insert(0, start)
    plt.plot(*zip(*order), 'b')
    plt.plot(start[0], start[1], 'ro', markersize=8)
    plt.show()


class AStarNode:
    def __init__(self, x=None, y=None, gw=1, hw=1, ew=1):
        self.open = True
        self.goal = False
        self.start = False
        self.x = x
        self.y = y
        self.parent = None
        self.e = 0. # e_cost: cost of total elevation traversed
        self.g = 0. # g_cost: cost from start to node (cost of the path)
        self.h = 0. # h_cost: cost of remaining distance to the goal (heuristic cost)
        self.f = 0. # f_cost: f = g*g_weight + h*h_weight + e*e_weight
        self.gw = gw # weight of g cost
        self.hw = hw # weight of h cost
        self.ew = ew # weight of e cost
        self.neighbors = []

    def calc_f(self):
        self.f =  self.g*self.gw \
                + self.h*self.hw \
                + self.e*self.ew

def visualize_path(path, cost_map, map_type='Slope', waypoints=None, explored_nodes=None):
    plt.imshow(cost_map, cmap='terrain')
    plt.colorbar(label=map_type)
    plt.title(map_type + ' Map')
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    #TODO: make the map bigger, add the text to the bottom of the map pop-up
    # between 9-10 and 3-4 for screen recording 
    legend = ['Start', 'Goal', 'Path']

    plt.plot(path[0][1], path[0][0], 'o', color='orange', markersize=8, label='Start', zorder=4)
    plt.plot(path[-1][1], path[-1][0], 'o', color='lime', markersize=8, label='Goal', zorder=4)
    plt.plot([x[1] for x in path], [x[0] for x in path], 'r-', zorder=2)
    
    if waypoints is not None:
        plt.scatter([x[1] for x in waypoints], [x[0] for x in waypoints], color='yellow', s=8, marker='o', zorder=3)
        legend.append('Waypoints')
    if explored_nodes is not None:

        plt.plot(path[0][1], path[0][0], 'o', color='black', markersize=8, label='Explored', zorder=0)

        plt.plot([x[1] for x in explored_nodes], [x[0] for x in explored_nodes], 'ko', markersize=1, alpha=0.08, zorder=1)
        legend.append('Explored')

    plt.legend(legend)

    plt.show()



def main():
    # Testing Waypoint Planning
    start = (12, 8)
    wp = [(5, 5), (12, 10), (8, 7), (6, 7), (10, 2), (11,5), (15, 10), (18, 8)]
    wp_ordered, _ = AStarPlanner.plan_wp_order(start, wp)
    visualize_plan_wp_order(start, wp_ordered)

if __name__ == '__main__':
    main()