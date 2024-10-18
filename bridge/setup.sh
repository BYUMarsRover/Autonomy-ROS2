#!/bin/bash
# Created by Nelson Durrant, Oct 2024

# Build everything but the ROS1 bridge
cd ~/bridge_ws
colcon build --symlink-install --packages-skip ros1_bridge

# Source ROS1 and ROS2 environments and build the ROS1 bridge
source /opt/ros/noetic/setup.bash
source ~/ros2_humble/install/setup.bash
colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure