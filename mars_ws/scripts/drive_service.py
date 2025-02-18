#!/usr/bin/env python3

import subprocess
import time
import os
import signal
import sys

# File to store the PID of this script
pid_file = "/tmp/ros2_launch_script.pid"

# Function to handle cleanup when the script is terminated
def signal_handler(sig, frame):
    print("Terminating ROS 2 nodes...")
    for pid in node_pids:
        try:
            os.kill(pid, signal.SIGTERM)  # Gracefully terminate each node
            print(f"Node with PID {pid} terminated.")
        except ProcessLookupError:
            print(f"Node with PID {pid} not found.")
    os.remove(pid_file)  # Remove PID file
    sys.exit(0)  # Exit the script

# Register signal handlers for SIGINT and SIGTERM
signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

# Store the PID of this script
with open(pid_file, 'w') as f:
    f.write(str(os.getpid()))

# Source the ROS 2 workspace
# subprocess.call('source /workspace/ros2_ws/install/setup.bash', shell=True)

# Start the ROS 2 launch process
launch_proc = subprocess.Popen(['ros2', 'launch', 'mobility', 'daemon_launch.py'])

# Wait for the launch process to start
time.sleep(2)

# Get the PIDs of the launched nodes
node_pids = []
for proc in subprocess.Popen(['ps', '--ppid', str(launch_proc.pid), '-o', 'pid='], stdout=subprocess.PIPE).stdout:
    node_pids.append(int(proc.strip()))

# Log the node PIDs
print(f"Tracking node PIDs: {node_pids}")

# Keep the script running and handle termination
try:
    launch_proc.wait()  # Wait for the launch process to finish
except Exception as e:
    print(f"Error while waiting for launch process: {e}")
finally:
    signal_handler(None, None)  # Ensure cleanup on exit
