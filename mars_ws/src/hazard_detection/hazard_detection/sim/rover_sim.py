import matplotlib.pyplot as plt
import numpy as np


import matplotlib.pyplot as plt
import numpy as np

class RoverVisualizer:
    def __init__(self):
        self.fig, self.ax = plt.subplots()
        self.rover = {'position': (0, 0), 'orientation': -np.pi / 2}  # Facing north (positive x)
        self.target = (5, 5)  # (x, y)
        self.obstacles = []  # List of (x, y, width, height)
    
    def draw(self):
        self.ax.clear()
        self.ax.set_xlim(-10, 10)
        self.ax.set_ylim(-10, 10)
        
        # Draw rover
        x, y = self.rover['position']
        theta = self.rover['orientation']
        width, height = 1.5, 1.0  # Rover dimensions
        
        # Compute rectangle corners
        rect = np.array([[-width/2, -height/2], [width/2, -height/2],
                         [width/2, height/2], [-width/2, height/2], [-width/2, -height/2]])
        R = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
        rect = np.dot(rect, R.T) + np.array([x, y])
        
        self.ax.plot(rect[:, 0], rect[:, 1], 'b')  # Rover body
        self.ax.plot(x, y, 'bo', markersize=5)  # Rover center
        
        # Draw small green dot at front of rover
        front_x = x + (width/2) * np.cos(theta)
        front_y = y + (width/2) * np.sin(theta)
        self.ax.plot(front_x, front_y, 'go', markersize=5)
        
        # Draw target
        self.ax.plot(self.target[0], self.target[1], 'ro', markersize=8)
        
        # Draw obstacles
        for ox, oy, w, h in self.obstacles:
            obstacle = plt.Rectangle((ox - w/2, oy - h/2), w, h, color='gray')
            self.ax.add_patch(obstacle)
        
        plt.draw()
    
    def update_display(self):
        """Updates the display without blocking execution."""
        self.draw()
        plt.pause(0.01)  # Allows real-time updates
    
    def set_rover_position(self, x, y, orientation):
        self.rover['position'] = (x, y)
        self.rover['orientation'] = orientation
        self.update_display()
    
    def set_target(self, x, y):
        self.target = (x, y)
        self.update_display()
    
    def add_obstacle(self, x, y, width=1, height=1):
        """Adds an obstacle to the environment."""
        self.obstacles.append((x, y, width, height))
        self.update_display()
    
    def remove_obstacle(self, x, y):
        self.obstacles = [(ox, oy, w, h) for ox, oy, w, h in self.obstacles if (ox, oy) != (x, y)]
        self.update_display()
    

    def start(self):
        """Starts the visualization (call this instead of plt.show())"""
        plt.ion()  # Turn on interactive mode
        self.draw()
        plt.show()

if __name__ == "__main__":
    visualizer = RoverVisualizer()
    visualizer.start()


# class RoverVisualizer:
#     def __init__(self):
#         self.fig, self.ax = plt.subplots()
#         self.rover = {
#             'position': (0, 0),
#             'orientation': -np.pi / 2  # Facing North (positive x)
#         }
#         self.target = (5, 5)
#         self.obstacles = []  # List of (x, y, width, height)
#         self.velocity = {'linear': 0, 'angular': 0}  # m/s, rad/s
#         self.dt = 0.1  # Time step for motion updates
        
#         plt.show()  # Start the event loop

#     def draw(self):
#         """Draws the entire scene including rover, target, and obstacles."""
#         self.ax.clear()
#         self.ax.set_xlim(-10, 10)
#         self.ax.set_ylim(-10, 10)
        
#         # Draw rover
#         x, y = self.rover['position']
#         theta = self.rover['orientation']
#         width, height = 1.5, 1.0  # Rover dimensions
        
#         # Compute rectangle corners
#         rect = np.array([[-width/2, -height/2], [width/2, -height/2],
#                          [width/2, height/2], [-width/2, height/2], [-width/2, -height/2]])
#         R = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
#         rect = np.dot(rect, R.T) + np.array([x, y])
        
#         self.ax.plot(rect[:, 0], rect[:, 1], 'b')  # Rover body
#         self.ax.plot(x, y, 'bo', markersize=5)  # Rover center
        
#         # Draw small green dot at front of rover
#         front_x = x + (width/2) * np.cos(theta)
#         front_y = y + (width/2) * np.sin(theta)
#         self.ax.plot(front_x, front_y, 'go', markersize=5)
        
#         # Draw target
#         self.ax.plot(self.target[0], self.target[1], 'ro', markersize=8)
        
#         # Draw obstacles
#         for ox, oy, w, h in self.obstacles:
#             obstacle = plt.Rectangle((ox - w/2, oy - h/2), w, h, color='gray')
#             self.ax.add_patch(obstacle)
        
#         plt.draw()

#     def update_display(self):
#         """Refreshes the visualization."""
#         self.draw()
#         plt.pause(0.01)

#     def set_rover_position(self, x, y, orientation):
#         """Manually sets rover position and orientation."""
#         self.rover['position'] = (x, y)
#         self.rover['orientation'] = orientation
#         self.update_display()

#     def set_target(self, x, y):
#         """Sets a new target position."""
#         self.target = (x, y)
#         self.update_display()

#     def add_obstacle(self, x, y, width=1, height=1):
#         """Adds an obstacle to the environment."""
#         self.obstacles.append((x, y, width, height))
#         self.update_display()

#     def set_velocity(self, linear, angular):
#         """Sets the linear (m/s) and angular (rad/s) velocity of the rover."""
#         self.velocity['linear'] = linear
#         self.velocity['angular'] = angular
#         self.update_display()

#     def update_position(self, frame):
#         """Updates the rover's position based on velocity."""
#         x, y = self.rover['position']
#         theta = self.rover['orientation']
        
#         # Update orientation using angular velocity
#         theta += self.velocity['angular'] * self.dt
        
#         # Update position using linear velocity
#         x += self.velocity['linear'] * np.cos(theta) * self.dt
#         y += self.velocity['linear'] * np.sin(theta) * self.dt

#         self.rover['position'] = (x, y)
#         self.rover['orientation'] = theta
#         self.draw()

