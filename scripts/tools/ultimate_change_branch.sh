#!/bin/bash

# Function to print error and exit
handle_error() {
    echo "Error: $1"
    exit 1
}

# Function to run a command and exit if it fails
run_command() {
    "$@"
    if [ $? -ne 0 ]; then
        handle_error "Command failed: $*"
    fi
}

# Function to display help message
show_help() {
    echo "Usage: ./ultimate_change_branch.sh <branch_name>"
    echo "This script checks out the specified Git branch locally and remotely,"
    echo "pulls the latest changes, and runs predefined build scripts on both."
    echo "This script must be run on the base station, and cannot be used to"
    echo "change to branches which predate this scripts addition."
    echo ""
    echo "Options:"
    echo "  -h, --help   Show this help message"
    exit 0
}

# Check for arguments
if [ -z "$1" ]; then
    handle_error "No branch name provided. Use --help or -h for usage information."
fi

# Handle --help or -h option
if [[ "$1" == "--help" || "$1" == "-h" ]]; then
    show_help
fi

# Store the initial directory
START_DIR="$(pwd)"

BRANCH_NAME="$1"
REMOTE_USER="marsrover"
REMOTE_HOST="192.168.1.120" 
REMOTE_SCRIPT="./catkin_rover_build.sh"
REMOTE_WORK_DIR="$HOME/BYU-Mars-Rover/rover_ws/"
LOCAL_SCRIPT="./catkin_base_build.sh"
LOCAL_WORK_DIR="$HOME/BYU-Mars-Rover/rover_ws"

# Change to the local working directory
echo "Changing to local working directory: $LOCAL_WORK_DIR"
cd "$LOCAL_WORK_DIR" || handle_error "Failed to change to directory: $LOCAL_WORK_DIR"

echo "Checking out branch '$BRANCH_NAME' locally..."
run_command git checkout "$BRANCH_NAME"

echo "Pulling latest changes locally..."
run_command git pull origin "$BRANCH_NAME"

# Run local script
if [ -f "$LOCAL_SCRIPT" ]; then
    echo "Running '$LOCAL_SCRIPT' locally..."
    run_command bash "$LOCAL_SCRIPT"
else
    handle_error "Local script '$LOCAL_SCRIPT' not found."
fi

# SSH into the remote device
echo "Connecting to remote device $REMOTE_USER@$REMOTE_HOST..."
run_command ssh "$REMOTE_USER@$REMOTE_HOST" bash -s <<EOF
    set -e  # Stop on error inside SSH session

    # Source the user's profile and bashrc
    echo "Sourcing ~/.bashrc and ~/.profile"
    source ~/.profile
    source ~/.bashrc

    # Change to the remote working directory
    echo "Changing to remote working directory: $REMOTE_WORK_DIR"
    cd "$REMOTE_WORK_DIR" || handle_error "Failed to change to directory: $REMOTE_WORK_DIR"

    # Checkout git branch
    echo "Checking out branch '$BRANCH_NAME' remotely..."
    git checkout "$BRANCH_NAME" || exit 1
    echo "Pulling latest changes remotely from base station..."
    git pull base "$BRANCH_NAME" || exit 

    # Run remote script
    if [ -f "$REMOTE_SCRIPT" ]; then
        echo "Running '$REMOTE_SCRIPT' remotely..."
        bash "$REMOTE_SCRIPT" || exit 1
    else
        echo "Error: Remote script '$REMOTE_SCRIPT' not found."
        exit 1
    fi
EOF

# Navigate back to the starting directory
cd "$START_DIR" || handle_error "Failed to return to directory: $START_DIR"

echo "Deployment completed successfully."
