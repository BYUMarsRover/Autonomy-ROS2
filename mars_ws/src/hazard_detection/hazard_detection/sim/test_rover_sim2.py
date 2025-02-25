from rover_sim2 import RoverSim
import time

sim = RoverSim()

# Start the simulation in a separate thread (optional for external control)
import threading
sim_thread = threading.Thread(target=sim.run, daemon=True)
sim_thread.start()

# Dynamically change velocity and angular velocity
sim.set_velocity(2, 0.5)  # Move forward with slight rotation
time.sleep(2)

sim.set_velocity(3, 0.5)  # Move faster with opposite rotation
time.sleep(2)

sim.set_velocity(0, 0)  # Stop the rover
