#!/bin/bash
# Created by Nelson Durrant, Oct 2024
#
# Tests the ROS1 bridge using the included ROS1 and ROS2 msg packages

# Create a new tmux session
tmux new-session -d -s bridge_session

# Create new panes
tmux split-window -h -t bridge_session
tmux split-window -v -t bridge_session
tmux split-window -v -t bridge_session
tmux select-layout -t bridge_session tiled



# Attach to the tmux session so we can see the output
tmux attach -t bridge_session && tmux kill-session -t bridge_session