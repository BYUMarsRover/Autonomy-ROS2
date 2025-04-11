from rover_sim import RoverVisualizer
import numpy as np
import matplotlib.pyplot as plt
import time

# Initialize the visualizer
visualizer = RoverVisualizer()

# Set a fixed target position
target_x, target_y = 5, 5
visualizer.set_target(target_x, target_y)

# Initial rover state
x, y, orientation = 0, 0, 0
linear_velocity = 0.5  # Adjust as needed
angular_velocity = 0.5  # Adjust as needed

dt = 0.1  # Time step

for i in range(100):
    # Update orientation based on angular velocity
    orientation += angular_velocity * dt
    
    # Update position based on linear velocity
    x += linear_velocity * np.cos(orientation) * dt
    y += linear_velocity * np.sin(orientation) * dt
    
    visualizer.set_rover_position(x, y, orientation)
    visualizer.update_display()
    
    time.sleep(dt)  # Simulate real-time updates

print("Simulation finished!")
