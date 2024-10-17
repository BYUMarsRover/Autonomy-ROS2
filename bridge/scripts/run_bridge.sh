#!/bin/bash
# Created by Nelson Durrant, Oct 2024
#
# Runs the ROS1 bridge

source /opt/ros/noetic/setup.bash
source ~/ros2_humble/install/setup.bash
source ~/ros1_msgs_ws/install_isolated/setup.bash
source ~/ros2_msgs_ws/install/local_setup.bash
source ~/bridge_ws/install/local_setup.bash
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics