#!/bin/bash

# Function to display usage information
usage() {
    echo "Usage: $0 [local|base]"
    exit 1
}

# Check if the correct number of arguments is provided
if [ "$#" -ne 1 ]; then
    usage
fi

# Set the IP address based on the argument
if [ "$1" == "local" ]; then
    IP_ADDRESS="127.0.0.1"
    export FASTRTPS_DEFAULT_PROFILES_FILE="/home/marsrover/mars_ws/super_rover_client_configuration_file.xml"
elif [ "$1" == "base" ]; then
    IP_ADDRESS="192.168.1.120"  
    export FASTRTPS_DEFAULT_PROFILES_FILE="/home/marsrover/mars_ws/super_base_client_configuration_file.xml"
else
    usage
fi

# Set up the ROS 2 CLI tools with Fast DDS
export ROS_DISCOVERY_SERVER="${IP_ADDRESS}:11811"

# Restart the ROS 2 daemon
ros2 daemon stop
ros2 daemon start

echo "ROS 2 CLI tools have been set up with Fast DDS using IP address ${IP_ADDRESS}."