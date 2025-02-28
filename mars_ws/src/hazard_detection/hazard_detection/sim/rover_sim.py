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
        self.orientation = np.pi/2 #rover's orientation from x, Positve being CCW
        self.x = 0 #rovers x position
        self.y = 0 #rovers y position

        # Initialize the figure and rover state
        self.fig, self.ax = plt.subplots(figsize=(12, 10))
        self.target = (5, 5)
        self.hazards = []

        self.init_parameter_display()

    # ---- Rover State Management ---- #
    
    def set_velocity(self, linear_vel, angular_vel, dt):
        """Updates the rover's velocity and position."""
        self.linear_vel = linear_vel
        self.angular_vel = angular_vel

        self.x = self.x + linear_vel * np.cos(self.orientation) * dt
        self.y = self.y + linear_vel * np.sin(self.orientation) * dt
        self.orientation = self.orientation - angular_vel * dt

        self.update_display()

    def set_rover_position(self, x, y, orientation):
        """Sets the rover's position and orientation."""
        self.x = x
        self.y = y
        self.orientation = orientation
        self.update_display()

    def get_rover_position(self):
        """Returns the rover's current position."""
        return (self.x, self.y)

    def get_rover_orientation(self):
        """Returns the rover's orientation as an angle from North."""
        return self.wrap(-(self.orientation - np.pi / 2), 0)

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
        
        self.update_target_info()

        params = [
            f"Linear Velocity: {self.linear_vel:.2f} m/s",
            f"Angular Velocity: {self.angular_vel:.2f} rad/s",
            f"Orientation: {self.get_rover_orientation():.2f} rad",
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
        width, height = 1.5, 1.0

        # Compute rectangle corners
        rect = np.array([
            [-width / 2, -height / 2], [width / 2, -height / 2],
            [width / 2, height / 2], [-width / 2, height / 2], 
            [-width / 2, -height / 2]
        ])
        R = np.array([[np.cos(self.orientation), -np.sin(self.orientation)], [np.sin(self.orientation), np.cos(self.orientation)]])
        rect = np.dot(rect, R.T) + np.array([self.x, self.y])

        self.ax.plot(rect[:, 0], rect[:, 1], 'b')
        self.ax.plot(self.x, self.y, 'bo', markersize=5)

        # Draw small green dot at the front
        front_x = self.x + (width / 2) * np.cos(self.orientation)
        front_y = self.y + (width / 2) * np.sin(self.orientation)
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

        self.course_angle = np.arctan2(self.target[0] - self.x, self.target[1] - self.y)
        self.distance_to_target = np.sqrt((self.target[0] - self.x)**2 + (self.target[1] - self.y)**2)
        self.course_heading_error = self.get_rover_orientation() - self.course_angle

    def set_target(self, x, y):
        """Sets the target position."""
        self.target = (x, y)
        self.update_target_info()
        self.update_display()

    def get_target(self):
        self.course_angle = np.arctan2(self.target[0] - self.x, self.target[1] - self.y)
        self.distance_to_target = np.sqrt((self.target[0] - self.x)**2 + (self.target[1] - self.y)**2)
         
        return self.distance_to_target, self.course_angle 

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

    def wrap(self, chi_1, chi_2):
        while chi_1 - chi_2 > np.pi:
            chi_1 -= 2.0 * np.pi
        while chi_1 - chi_2 < -np.pi:
            chi_1 += 2.0 * np.pi
        return chi_1

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
