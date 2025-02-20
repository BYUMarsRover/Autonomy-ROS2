#!/bin/bash
# Created by Nelson Durrant, Feb 2025
#
# Runs automatic commands on Docker container startup
# - WARNING! This script won't throw any errors, but if it
#   doesn't work, the container will crash immediately
# - Be very careful editing it

# Full color and mouse options
tmux set-option -g default-terminal "screen-256color"
tmux set -g mouse on

# Are we running on Jetson Orin architecture (the rover)?
if [ "$(uname -m)" == "aarch64" ]; then

    # Start a new 'rover_runtime' tmux session
    tmux new-session -d -s rover_runtime
    tmux send-keys -t rover_runtime.0 "clear" Enter

    # Launch ROS 2 nodes on system startup
    tmux send-keys -t rover_runtime.0 "ros2 launch mobility rover_xbox_launch.py" Enter
fi

# Start the SSH daemon in the Docker container
sudo /usr/sbin/sshd -D

# IMPORTANT! Keeps the container running
exec "$@"
