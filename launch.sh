#!/bin/bash
#
# Launches tasks over SSH using the 'rover_runtime' tmux session

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
DOCKER_SSH_PORT=2233

# Check for an SSH connection to the rover's Docker container
if ! ssh marsrover@$ROVER_IP_ADDRESS -p $DOCKER_SSH_PORT "echo" &> /dev/null
then
    printError "No available SSH connection to the rover's Docker container"
    echo "Here's some debugging suggestions:"
    echo "  - Make sure the SSH keys are setup by running the setup_ssh.sh script"
    echo "  - Ensure the rover is powered on"
    echo "  - Ensure the rover is connected with a static IP address"
    echo "  - Ensure the rover's Docker container is running"

    exit
fi

# Check if tmux is running on the rover's Docker container
if ! ssh marsrover@$ROVER_IP_ADDRESS -p $DOCKER_SSH_PORT "tmux has-session -t rover_runtime" &> /dev/null
then
    printError "No tmux session found in the rover's Docker container"
    echo "Here's some debugging suggestions:"
    echo "  - Ensure the rover's Docker container is running the 'rover_runtime' tmux session"

    exit
fi

#TODO FIX THIS LATER

# # Check that only one window is open in the 'rover_runtime' tmux session
# if [ $(ssh marsrover@$ROVER_IP_ADDRESS -p $DOCKER_SSH_PORT "tmux list-windows -t rover_runtime | wc -l") -ne 1 ]
# then
#     printWarning "Multiple windows found in the 'rover_runtime' tmux session"
#     echo "Simply entering the current tmux session for cleanup..."
#     ssh -t -X marsrover@$ROVER_IP_ADDRESS -p $DOCKER_SSH_PORT 'tmux attach -t rover_runtime'

#     exit
# fi

# Check that only one pane is open in the 'rover_runtime' tmux session
if [ $(ssh marsrover@$ROVER_IP_ADDRESS -p $DOCKER_SSH_PORT "tmux list-panes -t rover_runtime | wc -l") -ne 1 ]
then
    printWarning "Multiple panes found in the 'rover_runtime' tmux session"
    echo "Simply entering the current tmux session for cleanup..."
    ssh -t marsrover@$ROVER_IP_ADDRESS -p $DOCKER_SSH_PORT 'tmux attach -t rover_runtime'

    exit
fi

# Launch the specified task configuration over SSH
case "$1" in
    "autonomy")
        printInfo "Setting up the autonomy task..."
        # Send tmux commands to the rover's Docker container over SSH
        ssh marsrover@$ROVER_IP_ADDRESS -p $DOCKER_SSH_PORT "\
            tmux split-window -h -t rover_runtime:0.0; \
            tmux select-pane -t rover_runtime:0.1; \
            tmux send-keys -t rover_runtime:0.1 'export ROS_DISCOVERY_SERVER=127.0.0.1:11811' Enter; \
            tmux send-keys -t rover_runtime:0.1 'ros2 launch start rover_task_autonomy_new_launch.py'"
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
        printWarning "No task specified, simply entering the current tmux session..."
        echo "Specify a task using 'bash launch.sh <task>' "
        ;;
esac

# Attach to the 'rover_runtime' tmux session
ssh -t marsrover@$ROVER_IP_ADDRESS -p $DOCKER_SSH_PORT 'tmux attach -t rover_runtime'
