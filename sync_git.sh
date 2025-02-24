#!/bin/bash
# Created by Nelson Durrant, Feb 2025
#
# Pulls the base station git branch and changes onto the rover

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

# Get the current branch name
current_branch=$(git branch --show-current)

# Check for an SSH connection to the rover
if ! ssh marsrover@$ROVER_IP_ADDRESS "echo" &> /dev/null
then
    printError "No available SSH connection to the rover's computer"
    echo "Here's some debugging suggestions:"
    echo "  - Ensure the rover is powered on"
    echo "  - Ensure the rover is connected with a static IP address"

    exit
fi

# Connect to the rover and pull the base station branch
ssh marsrover@$ROVER_IP_ADDRESS "cd ~/Autonomy-ROS2; \
    git checkout $current_branch; \
    git pull base $current_branch"
