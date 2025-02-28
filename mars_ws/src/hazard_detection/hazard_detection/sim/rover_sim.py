import matplotlib.pyplot as plt
import numpy as np
import time

class RoverVisualizer:
    def __init__(self):
        """Initialize the rover visualization."""
        self.linear_vel = 0
        self.angular_vel = 0
        self.distance_to_target = 0.0
        self.course_angle = 0.0
        self.course_heading_error = 0.0
        self.orientation = 0.0

        # Initialize the figure and rover state
        self.fig, self.ax = plt.subplots()
        self.rover = {'position': (0, 0), 'orientation': np.pi / 2}  # Facing north (+y)
        self.target = (5, 5)
        self.hazards = []

        self.init_parameter_display()

    # ---- Rover State Management ---- #
    
    def set_velocity(self, linear_vel, angular_vel, dt):
        """Updates the rover's velocity and position."""
        self.linear_vel = linear_vel
        self.angular_vel = angular_vel

        x, y = self.rover['position']
        orientation = self.rover['orientation']

        new_x = x + linear_vel * np.cos(orientation) * dt
        new_y = y + linear_vel * np.sin(orientation) * dt
        new_orientation = orientation + angular_vel * dt

        self.set_rover_position(new_x, new_y, new_orientation)

    def set_rover_position(self, x, y, orientation):
        """Sets the rover's position and orientation."""
        self.rover['position'] = (x, y)
        self.rover['orientation'] = orientation
        self.update_display()

    def get_rover_position(self):
        """Returns the rover's current position."""
        return self.rover['position']

    def get_rover_orientation(self):
        """Returns the rover's orientation as an angle from North."""
        return -(self.rover['orientation'] - np.pi / 2)

    # ---- Display and Drawing ---- #

    def init_parameter_display(self):
        """Initializes text display for rover parameters."""
        self.param_texts = []
        y_positions = np.linspace(0.9, 0.65, 6)  # Evenly space text positions

        labels = [
            "Linear Velocity: 0.0 m/s",
            "Angular Velocity: 0.0 rad/s",
            "Orientation: 0.0 rad",
            "Course Angle: 0.0 rad",
            "Course Heading Error: 0.0 rad",
            "Distance to Target: 0.0 m"
        ]

        for y, label in zip(y_positions, labels):
            self.param_texts.append(
                self.fig.text(0.75, y, label, fontsize=12, bbox=dict(facecolor='white', edgecolor='black'))
            )

    def update_parameter_display(self):
        """Updates the displayed rover parameters."""
        params = [
            f"Linear Velocity: {self.linear_vel:.2f} m/s",
            f"Angular Velocity: {self.angular_vel:.2f} rad/s",
            f"Orientation: {self.orientation:.2f} rad",
            f"Course Angle: {self.course_angle:.2f} rad",
            f"Course Heading Error: {self.course_heading_error:.2f} rad",
            f"Distance to Target: {self.distance_to_target:.2f} m"
        ]

        for text, value in zip(self.param_texts, params):
            text.set_text(value)

        plt.draw()

    def draw(self):
        """Redraws the simulation environment."""
        self.ax.clear()
        self.ax.set_xlim(-10, 10)
        self.ax.set_ylim(-10, 10)
        self.update_target_info()
        self.update_parameter_display()

        # Draw the rover
        x, y = self.rover['position']
        theta = self.rover['orientation']
        width, height = 1.5, 1.0

        # Compute rectangle corners
        rect = np.array([
            [-width / 2, -height / 2], [width / 2, -height / 2],
            [width / 2, height / 2], [-width / 2, height / 2], 
            [-width / 2, -height / 2]
        ])
        R = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
        rect = np.dot(rect, R.T) + np.array([x, y])

        self.ax.plot(rect[:, 0], rect[:, 1], 'b')
        self.ax.plot(x, y, 'bo', markersize=5)

        # Draw small green dot at the front
        front_x = x + (width / 2) * np.cos(theta)
        front_y = y + (width / 2) * np.sin(theta)
        self.ax.plot(front_x, front_y, 'go', markersize=5)

        # Draw target
        self.ax.plot(self.target[0], self.target[1], 'ro', markersize=8)

        # Draw hazards
        for ox, oy, w, h in self.hazards:
            self.ax.add_patch(plt.Rectangle((ox - w / 2, oy - h / 2), w, h, color='gray'))

        plt.draw()

    def update_display(self):
        """Updates the display without blocking execution."""
        self.draw()
        plt.pause(0.01)

    # ---- Target and Hazard Management ---- #

    def update_target_info(self):
        """Updates distance, course angle, and heading error."""
        x, y = self.rover['position']
        self.orientation = self.rover['orientation']
        self.distance_to_target = np.hypot(self.target[0] - x, self.target[1] - y)
        self.course_angle = np.arctan2(self.target[1] - y, self.target[0] - x) # might need to look at doing this from the green dot, not blue one
        self.course_heading_error = self.orientation - self.course_angle

    def set_target(self, x, y):
        """Sets the target position."""
        self.target = (x, y)
        self.update_target_info()
        self.update_display()

    def add_hazard(self, x, y, width=1, height=1):
        """Adds a hazard to the environment."""
        self.hazards.append((x, y, width, height))
        self.update_display()

    def get_hazard_locations(self):
        """Returns a list of (x, y) positions of all hazards."""
        # for one hazard
        if len(self.hazards) > 1:
            return [(hx, hy) for hx, hy, _, _ in self.hazards] # multiple hazards
        elif len(self.hazards) == 1:
            hx, hy, _, _ = self.hazards[0]

            return hx, hy
        
            # #return the hazard in the rover frame
            # haz_x = hx - self.rover['position'][0]
            # haz_y = hy - self.rover['position'][1]
            # #rotate the hazard to the rover frame
            # haz_x = haz_x * np.cos(self.rover['orientation']) + haz_y * np.sin(self.rover['orientation'])
            # haz_y = -haz_x * np.sin(self.rover['orientation']) + haz_y * np.cos(self.rover['orientation'])

            return haz_x, haz_y
        else:
            return None
            
    def remove_hazard(self, x, y):
        """Removes a hazard at a specified location."""
        self.hazards = [(ox, oy, w, h) for ox, oy, w, h in self.hazards if (ox, oy) != (x, y)]
        self.update_display()

    # ---- Simulation Control ---- #

    def start(self):
        """Starts the visualization."""
        plt.ion()
        self.draw()
        plt.show()

if __name__ == "__main__":
    visualizer = RoverVisualizer()
    visualizer.start()

    # Set a target
    visualizer.set_target(5, 5)

    # Simulate movement
    x, y, orientation = 0, 0, 0
    linear_velocity = 0.5
    angular_velocity = 0.5
    dt = 0.5

    for _ in range(200):
        visualizer.set_velocity(linear_velocity, angular_velocity, dt)
        orientation += angular_velocity * dt
        x += linear_velocity * np.cos(orientation) * dt
        y += linear_velocity * np.sin(orientation) * dt

        # linear_velocity = linear_velocity - .1

        visualizer.set_rover_position(x, y, orientation)
        time.sleep(dt)

    print("Simulation finished!")


# import matplotlib.pyplot as plt
# import numpy as np
# import time

# class RoverVisualizer:
#     def __init__(self):
#         self.linear_vel = 0
#         self.angular_vel = 0
#         self.fig, self.ax = plt.subplots()
#         self.rover = {'position': (0, 0), 'orientation': np.pi / 2}  # Facing north (positive y) 
#         self.target = (5, 5)  # (x, y)
#         self.hazards = []  # List of (x, y, width, height)
#         self.distance_to_target = 0.0
#         self.course_angle = 0.0
#         self.course_heading_error = 0.0
#         self.orientation = 0.0
#         self.init_parameter_display()

#     def set_velocity(self, linear_vel, angular_vel, dt):
#         self.linear_vel = linear_vel
#         self.angular_vel = angular_vel
#         self.x = self.rover['position'][0]
#         self.y = self.rover['position'][1]
#         self.orientation = self.rover['orientation']
#         self.set_rover_position(self.x + self.linear_vel * np.cos(self.orientation) * dt, 
#                                 self.y + self.linear_vel* np.sin(self.orientation) * dt, self.orientation + self.angular_vel * dt)
    
#     def init_parameter_display(self):
#         self.param_texts = []
#         y_positions = [0.9, 0.85, 0.8, 0.75, 0.7, 0.65]  # Spaced out Y positions

#         labels = [
#             "Linear Velocity: 0.0 m/s",
#             "Angular Velocity: 0.0 rad/s",
#             "Orientation: 0.0 rad",
#             "Course Angle: 0.0 rad",
#             "Course Heading Error: 0.0 rad",
#             "Distance to Target: 0.0 m"
#         ]

#         for i in range(len(labels)):
#             self.param_texts.append(
#                 self.fig.text(0.75, y_positions[i], labels[i], fontsize=12, 
#                             bbox=dict(facecolor='white', edgecolor='black'))
#             )

#     def update_parameter_display(self):
#         self.param_texts[0].set_text(f"Linear Velocity: {self.linear_vel:.2f} m/s")
#         self.param_texts[1].set_text(f"Angular Velocity: {self.angular_vel:.2f} rad/s")
#         self.param_texts[2].set_text(f"Orientation: {self.orientation:.2f} rad")
#         self.param_texts[3].set_text(f"Course Angle: {self.course_angle:.2f} rad")
#         self.param_texts[4].set_text(f"Course Heading Error: {self.course_heading_error:.2f} rad")
#         self.param_texts[5].set_text(f"Distance to Target: {self.distance_to_target:.2f} m")

#         plt.draw()

            
#     def draw(self):
#         self.ax.clear()
#         self.ax.set_xlim(-10, 10)
#         self.ax.set_ylim(-10, 10)
#         self.get_target() # update distance to target, course angle, and course heading error
#         self.update_parameter_display()
        
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
        
#         # Draw hazards
#         for ox, oy, w, h in self.hazards:
#             hazard = plt.Rectangle((ox - w/2, oy - h/2), w, h, color='gray')
#             self.ax.add_patch(hazard)

#         plt.draw()
    
#     def update_display(self):
#         """Updates the display without blocking execution."""
#         self.draw()
#         plt.pause(0.01)  # Allows real-time updates

#     def get_target(self):
#         self.get_rover_position() # update rover position so that self.y != 0 accidently
#         if self.y == 0:
#             self.course_angle = 0.0
#         else:
#             self.course_angle = np.arctan(self.x/self.y) + np.pi / 2 # test this value
#         self.distance_to_target = np.sqrt((self.target[0] - self.rover['position'][0])**2 + (self.target[1] - self.rover['position'][1])**2)
#         self.course_heading_error = self.orientation - self.course_angle - np.pi / 2
#         self.orientation = -(self.rover['orientation'] - np.pi/2) 
#         return self.distance_to_target, self.course_angle 

#     def get_rover_orientation(self):
#         #return as an angle from North, need to subtract pi/2
#         return -(self.rover['orientation'] - np.pi/2)
    
#     def set_rover_position(self, x, y, orientation):
#         self.rover['position'] = (x, y)
#         self.rover['orientation'] = orientation
#         self.update_display()

#     def get_rover_position(self):
#         self.x = self.rover['position'][0]
#         self.y = self.rover['position'][1]
#         return self.x, self.y
    
#     def set_target(self, x, y, distance_to_target, course_angle):
#         self.distance_to_target = distance_to_target
#         self.course_angle = course_angle
#         self.course_heading_error = self.orientation - self.course_angle - np.pi / 2 
#         #have to subtract 90 deg to get orientation and course angle in same reference frame
#         self.target = (x, y)
#         self.update_display()
    
#     def add_hazard(self, x, y, width=1, height=1):
#         """Adds an hazard to the environment."""
#         self.hazards.append((x, y, width, height))
#         self.update_display()
    
#     def get_hazard_locations(self):
#         """Returns a list of (x, y) positions of all hazards."""
#         # for one hazard
#         if len(self.hazards) > 1:
#             return [(hx, hy) for hx, hy, _, _ in self.hazards] # multiple hazards
#         elif len(self.hazards) == 1:
#             hx, hy, _, _ = self.hazards[0]
#             return hx, hy
#         else:
#             return None
            
#     def remove_hazard(self, x, y):
#         self.hazards = [(ox, oy, w, h) for ox, oy, w, h in self.hazards if (ox, oy) != (x, y)]
#         self.update_display()

#     def start(self):
#         """Starts the visualization (call this instead of plt.show())"""
#         plt.ion()  # Turn on interactive mode
#         self.draw()
#         plt.show()

# if __name__ == "__main__":
#     visualizer = RoverVisualizer()

#     # Initial rover state
#     x, y, orientation = 0, 0, 0
#     linear_velocity = 0.5  # Adjust as needed
#     angular_velocity = 0.5  # Adjust as needed\
#     course_angle = 0.0
#     distance_to_target = 10
#     dt = 0.1
#     visualizer.set_velocity(linear_velocity, angular_velocity, dt)    

#     visualizer.start()
#     # Set a fixed target position
#     target_x, target_y = 5, 5
#     visualizer.set_target(target_x, target_y, distance_to_target, course_angle)

#     dt = 0.1  # Time step

#     for i in range(100):
#         # Update orientation based on angular velocity
#         orientation += angular_velocity * dt
        
#         # Update position based on linear velocity
#         x += linear_velocity * np.cos(orientation) * dt
#         y += linear_velocity * np.sin(orientation) * dt
        
#         visualizer.set_rover_position(x, y, orientation)
#         visualizer.update_display()
        
#         time.sleep(dt)  # Simulate real-time updates

#     print("Simulation finished!")