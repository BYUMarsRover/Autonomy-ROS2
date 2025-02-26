import matplotlib.pyplot as plt
import numpy as np

class RoverVisualizer:
    def __init__(self):
        self.fig, self.ax = plt.subplots()
        self.rover = {'position': (0, 0), 'orientation': -np.pi / 2}  # Facing north (positive x)
        self.target = (5, 5)  # (x, y)
        self.hazards = []  # List of (x, y, width, height)
        self.init_velocity_display()

    def set_velocity(self, linear_vel, angular_vel, dt):
        x += linear_vel * np.cos(orientation) * dt
        y += linear_vel* np.sin(orientation) * dt
        orientation += angular_vel * self.dt
        self.set_rover_position(x, y, orientation)
        self.update_velocity_display(linear_vel, angular_vel)
    
    def init_velocity_display(self):
        self.velocity_texts.append(self.fig.text(0.75, 0.9, "Linear Velocity: 0.0 m/s", fontsize=12, bbox=dict(facecolor='white', edgecolor='black')))
        self.velocity_texts.append(self.fig.text(0.75, 0.85, "Angular Velocity: 0.0 rad/s", fontsize=12, bbox=dict(facecolor='white', edgecolor='black')))
    
    def update_velocity_display(self, linear, angular):
        self.linear_velocity = linear
        self.angular_velocity = angular
        self.velocity_texts[0].set_text(f"Linear Velocity: {linear:.2f} m/s")
        self.velocity_texts[1].set_text(f"Angular Velocity: {angular:.2f} rad/s")
        plt.draw()
            
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
        
        # Draw hazards
        for ox, oy, w, h in self.hazards:
            hazard = plt.Rectangle((ox - w/2, oy - h/2), w, h, color='gray')
            self.ax.add_patch(hazard)

        plt.draw()
    
    def update_display(self):
        """Updates the display without blocking execution."""
        self.draw()
        plt.pause(0.01)  # Allows real-time updates

    def get_hazard_locations():
        return self.hazards[0]

    def get_target(self):
        return np.sqrt((self.target[0] - self.rover['position'][0])**2 + (self.target[1] - self.rover['position'][1])**2)

    def get_rover_orientation(self):
        return self.rover['orientation']
    
    def set_rover_position(self, x, y, orientation):
        self.rover['position'] = (x, y)
        self.rover['orientation'] = orientation
        self.update_display()

    def get_rover_position(self):
        return self.rover['position'][0], self.rover['position'][1]
    
    def set_target(self, x, y):
        self.target = (x, y)
        self.update_display()
    
    def get_hazard_locations(self):
        """Returns a list of (x, y) positions of all hazards."""
        # for one hazard
        if self.hazards.length > 1:
            return [(hx, hy) for hx, hy, _, _ in self.hazards] # multiple hazards
        elif self.hazards.length == 1:
            hx, hy, _, _ = self.hazards[0]
            return hx, hy
        else:
            return None
            
    def remove_hazard(self, x, y):
        self.hazards = [(ox, oy, w, h) for ox, oy, w, h in self.hazards if (ox, oy) != (x, y)]
        self.update_display()

    def start(self):
        """Starts the visualization (call this instead of plt.show())"""
        plt.ion()  # Turn on interactive mode
        self.draw()
        plt.show()

if __name__ == "__main__":
    visualizer = RoverVisualizer()
    visualizer.start()