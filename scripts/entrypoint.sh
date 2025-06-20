#!/bin/bash
# Created by Nelson Durrant & Braden Meyers, Feb 2025
#
# Runs automatic commands on Docker container startup
# - This script won't throw any errors, but the container will crash immediately
# - Be very careful editing it!

# Are we running on Jetson Orin architecture (the rover)?

# if [ "$(uname -m)" == "aarch64" ]; then

# Changed for the new laptop!
# Start a new 'rover_runtime' tmux session
tmux new-session -d -s rover_runtime

# Send 'clear' command to the first window
tmux send-keys -t rover_runtime:0 "clear" Enter
# Full color and mouse options
tmux set-option -g default-terminal "screen-256color"
tmux set -g mouse on
# Create a new window within the 'rover_runtime' session and name it 'fastdds_server'
tmux new-window -t rover_runtime -n fastdds_server

# Send the 'fastdds discovery' command to the 'fastdds_server' window
tmux send-keys -t rover_runtime:fastdds_server "fastdds discovery --server-id 0" Enter

#let the discovery server startup before adding nodes
sleep 3

# Launch ROS 2 nodes on system startup

tmux send-keys -t rover_runtime:0.0 "export ROS_DISCOVERY_SERVER=127.0.0.1:11811" Enter
tmux send-keys -t rover_runtime:0.0 "ros2 launch start rover_usb_devices_launch.py"

# fi

# Start the SSH daemon in the Docker container
sudo /usr/sbin/sshd -D

# IMPORTANT! Keeps the container running
exec "$@"
