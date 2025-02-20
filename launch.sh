#!/bin/bash
# Created by Nelson Durrant, Feb 2025
#
# Launches tasks over SSH using the 'rover_runtime' tmux session
# - This allows us to stream the GUI from inside the Docker container

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

# Check for an SSH connection to the rover's Docker container
if ! ssh marsrover@192.168.1.120 -p 2233 "echo 'SSH connection successful'" &> /dev/null
then
    printError "No available SSH connection to the rover's Docker container"
    echo "Here's some debugging suggestions:"
    echo "  - Ensure the rover is powered on"
    echo "  - Ensure the rover is connected to the right network"
    echo "  - Ensure the rover's Docker container is running"

    exit
fi

# Check if tmux is running on the rover's Docker container
if ! ssh marsrover@192.168.1.120 -p 2233 "tmux has-session -t rover_runtime" &> /dev/null
then
    printError "No tmux session found in the rover's Docker container"
    echo "Here's some debugging suggestions:"
    echo "  - Ensure the rover's Docker container is running the 'rover_runtime' tmux session"

    exit
fi

# Launch the specified task configuration over SSH
case "$1" in
    "autonomy")
        printInfo "Setting up the autonomy task..."
        # Send tmux commands to the rover's Docker container over SSH
        sshpass -p "marsrover" ssh marsrover@192.168.1.120 -p 2233 "\
            tmux split-window -h -t rover_runtime; \
            tmux select-pane -t rover_runtime.1; \
            tmux send-keys -t rover_runtime.1 'ros2 launch start rover_task_autonomy_new_launch.py'" # NO ENTER 
        ;;
    "servicing")
        printWarning "Not implemented yet"
        ;;
    "retrieval")
        printWarning "Not implemented yet"
        ;;
    "science")
        printWarning "Not implemented yet"
        ;;
    *)
        printError "No task specified"
        printError "Specify a task using 'bash launch.sh <task>' (ex. 'bash launch.sh autonomy')"
        exit
        ;;
esac

# Attach to the 'rover_runtime' tmux session
ssh -t marsrover@192.168.1.120 -p 2233 'tmux attach -t rover_runtime'
