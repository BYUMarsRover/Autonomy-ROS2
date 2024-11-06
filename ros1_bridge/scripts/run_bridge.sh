#!/bin/bash
# Created by Nelson Durrant, Oct 2024
#
# Runs the ROS 1 bridge

# Run 'echo $ROS_MASTER_URI' from a sourced, running ROS 1 terminal 
# from outside the Docker image to get the IP and/or port address to add here
export ROS_MASTER_URI=http://192.168.1.120:11311

source /opt/ros/noetic/setup.bash
source ~/ros1_msgs_ws/install_isolated/setup.bash
source ~/ros2_humble/install/setup.bash
source ~/ros2_msgs_ws/install/local_setup.bash
source ~/bridge_ws/install/local_setup.bash
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics