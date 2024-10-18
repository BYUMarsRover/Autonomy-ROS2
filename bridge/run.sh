#!/bin/bash
# Created by Nelson Durrant, Oct 2024

# Run roscore in the background
source /opt/ros/noetic/setup.bash
roscore &

# Source ALL the environments
source /opt/ros/noetic/setup.bash
source ~/ros2_humble/install/setup.bash
source ~/bridge_ws/install/setup.bash

# Connect and run the bridge
export ROS_MASTER_URI=http://localhost:11311
ros2 run ros1_bridge dynamic_bridge