#!/usr/bin/env bash

###############################################################################
# Set the default rover address
###############################################################################
ROVER_ADDRESS="192.168.1.120"
TIMEOUT=5

# Base station ROS environment
BASE_ENVIRONMENT="
ROS_MASTER_URI=http://$ROVER_ADDRESS:11311
ROS_IP=localhost
ROVER_ADDRESS=$ROVER_ADDRESS
"

# Rover ROS environment
ROVER_ENVIRONMENT="
ROS_MASTER_URI=http://$ROVER_ADDRESS:11311
ROS_IP=$ROVER_ADDRESS
ROVER_ADDRESS=$ROVER_ADDRESS
"

# Prepare to set up environment variables
SET_BASE_ENV_CMD="export $(echo $BASE_ENVIRONMENT | xargs) && unset ROS_HOSTNAME && source /path/to/base_station_workspace/devel/setup.sh"
SET_ROVER_ENV_CMD="export $(echo $ROVER_ENVIRONMENT | xargs) && unset ROS_HOSTNAME && source /path/to/rover_workspace/devel/setup.sh"

# Function to run commands on the rover
function rover_cmd {
  ssh $ROVER_ADDRESS -o ConnectTimeout=$TIMEOUT "$SET_ROVER_ENV_CMD && $1" || {
    echo "[ERROR] Unable to connect to rover at $ROVER_ADDRESS"
    exit 1
  }
}

# Start the base station launch file locally
# echo "[INFO] Launching base station ROS node locally"
# $SET_BASE_ENV_CMD && ros2 launch keyboard_autonomy_base_launch.py

# Start rover's Docker container and launch the rover node
echo "[INFO] Launching rover ROS node remotely"
rover_cmd "cd ~/Autonomy-ROS2 && ./compose.sh"
rover_cmd "$SET_ROVER_ENV_CMD && ros2 launch keyboard_autonomy_rover_launch.py"