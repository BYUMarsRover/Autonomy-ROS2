#!/bin/bash
# Created by Braden Meyers, Feb 2025
#
# Launches the zed driver over SSH using tmux

function printInfo {
  # print blue
  echo -e "\033[0m\033[36m[INFO] $1\033[0m"
}

function printWarning {
  # print yellow
  echo -e "\033[0m\033[33m[WARNING] $1\033[0m"
}

function printError {
  # print red
  echo -e "\033[0m\033[31m[ERROR] $1\033[0m"
}

ROVER_IP_ADDRESS=192.168.1.120

# Check for an SSH connection to the rover's Docker container
if ! ssh marsrover@$ROVER_IP_ADDRESS "echo" &> /dev/null
then
    printError "No available SSH connection to the rover's computer"
    echo "Here's some debugging suggestions:"
    echo "  - Ensure the rover is powered on"
    echo "  - Ensure the rover is connected with a static IP address"
    echo "  - Ensure the rover's computer has a green light"
    echo "  - Ensure the rocket antennas are connected"

    exit
fi

# Check if tmux is running on the rover computer
if ! ssh marsrover@$ROVER_IP_ADDRESS "tmux has-session -t foxy_runtime" &> /dev/null
then
    printInfo "Starting the ZED tmux session..."
    # Send tmux commands to the rover's Docker container over SSH
    ssh marsrover@$ROVER_IP_ADDRESS "tmux new-session -d -s foxy_runtime; \
    tmux send-keys -t foxy_runtime.0 'clear' Enter;\
    tmux send-keys -t foxy_runtime.0 'source /opt/ros/foxy/setup.bash' Enter; \
    tmux send-keys -t foxy_runtime.0 'cd ~/foxy_ws && source install/setup.bash' Enter; \
    tmux send-keys -t foxy_runtime.0 'ros2 launch object_detection object_detection_launch.py'" # NO ENTER
else
    printWarning "ZED code already running, simply entering the current tmux session"
fi

ssh -t -X marsrover@$ROVER_IP_ADDRESS "tmux attach -t foxy_runtime"
