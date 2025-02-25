
from rover_sim import RoverVisualizer
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time
import numpy as np

# Initialize the visualizer
visualizer = RoverVisualizer()

# Example: Update position and target dynamically
for i in range(100):
    x = np.sin(i * 0.1) * 5  # Simulated movement in a sine wave
    y = np.cos(i * 0.1) * 5
    orientation = i * 0.1  # Incremental rotation

    visualizer.set_rover_position(x, y, orientation)
    visualizer.set_target(x + 2, y + 2)  # Move target dynamically

    visualizer.update_display()  # Refresh the visualization
    time.sleep(0.01)  # Simulate real-time updates

print("Simulation finished!")



# # Initialize the visualizer
# visualizer = RoverVisualizer()

# # Set an initial velocity (1 m/s forward, 0.5 rad/s rotation)
# visualizer.set_velocity(1.0, 0.5)

# # Add obstacles
# visualizer.add_obstacle(2, 3, width=2, height=1)
# visualizer.add_obstacle(-4, -2, width=1.5, height=2)

# # Let it move for 5 seconds
# time.sleep(5)

# # Change velocity
# visualizer.set_velocity(0.5, -0.2)

# # Run for another 5 seconds
# time.sleep(5)

# # Stop the rover
# visualizer.set_velocity(0, 0)
