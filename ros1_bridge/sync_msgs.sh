#!/bin/bash
# Created by Nelson Durrant, Oct 2024
#
# Syncs the messages in ros2_msgs_ws with the messages in the general ROS2 repository

rsync -avphd ~/Autonomy-ROS2/mars_ws/src/rover_msgs ~/Autonomy-ROS2/ros1_bridge/ros2_msgs_ws/src