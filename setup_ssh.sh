#!/bin/bash

# Function to remove an old SSH key from known_hosts
# This prevents SSH errors if the host's key has changed.
remove_old_ssh_key() {
    local host=$1
    local port=$2

    if [[ "$port" == "22" ]]; then
        # Remove without specifying port notation for default SSH port
        ssh-keygen -f "$HOME/.ssh/known_hosts" -R "$host" &> /dev/null
    else
        # Remove using port notation for non-default ports
        ssh-keygen -f "$HOME/.ssh/known_hosts" -R "[$host]:$port" &> /dev/null
    fi

    echo "Removed old SSH key for $host on port $port"
}

# Prompt the user for where they want to send the SSH key
echo "Do you want to send the key to the rover, the rover's Docker container, or this computer's Docker container?"
echo "1) Rover (physical machine)"
echo "2) Rover Docker Container (running inside the rover)"
echo "3) This Computer's Docker Container (e.g., Base Station)"
echo "4) All of the above"
read -p "Enter your choice (1/2/3/4): " choice

# Define the target IP address of the rover and the port for the Docker container
ROVER_IP_ADDRESS="192.168.1.120"  # The rover's network address
DOCKER_SSH_PORT="2233"  # The SSH port used by the Docker container on the rover

DOCKER_PWD="marsrover"  # The password for the Docker container
ROVER_PWD="thekillpack"  # The password for the rover
BASE_PWD="thekillpack"  # The password for the base station

#Check if ssh pass is installed
if ! command -v sshpass &> /dev/null; then
    echo "Error: sshpass is not installed. Please install it using (sudo apt install sshpass)."
    DOCKER_CONNECT="ssh"
    ROVER_CONNECT="ssh"
    BASE_CONNECT="ssh"
else
    DOCKER_CONNECT="sshpass -p $DOCKER_PWD ssh"
    ROVER_CONNECT="sshpass -p $ROVER_PWD ssh"
    BASE_CONNECT="sshpass -p $BASE_PWD ssh"
fi


# Process the user's selection
case $choice in
    1)
        # Remove old SSH key (if it exists) to prevent key mismatch issues
        remove_old_ssh_key "$ROVER_IP_ADDRESS" "22"
        echo "here"

        # Test the SSH connection to accept the new key if prompted
        $ROVER_CONNECT -o StrictHostKeyChecking=accept-new marsrover@$ROVER_IP_ADDRESS "echo" &> /dev/null
        echo "here"

        # Copy the SSH key to the rover (physical machine)
        ssh-copy-id marsrover@$ROVER_IP_ADDRESS
        ;;
    2)
        # Remove old SSH key for the Docker container
        remove_old_ssh_key "$ROVER_IP_ADDRESS" "$DOCKER_SSH_PORT"

        # Test the connection to the Docker container and accept the new key
        $DOCKER_CONNECT -o StrictHostKeyChecking=accept-new marsrover@$ROVER_IP_ADDRESS -p $DOCKER_SSH_PORT "echo" &> /dev/null

        # Copy the SSH key to the rover's Docker container
        ssh-copy-id -p $DOCKER_SSH_PORT marsrover@$ROVER_IP_ADDRESS
        ;;
    3)
        # Remove old SSH key for this computer's Docker container
        remove_old_ssh_key "localhost" "$DOCKER_SSH_PORT"

        # Test the connection to accept the new key
        $BASE_CONNECT -o StrictHostKeyChecking=accept-new marsrover@localhost -p $DOCKER_SSH_PORT "echo" &> /dev/null

        # Copy the SSH key to the local Docker container (running on this machine)
        ssh-copy-id -p $DOCKER_SSH_PORT marsrover@localhost

        ;;
    4)
        # Remove old SSH keys from all locations to avoid connection issues
        remove_old_ssh_key "$ROVER_IP_ADDRESS" "22"
        remove_old_ssh_key "$ROVER_IP_ADDRESS" "$DOCKER_SSH_PORT"
        remove_old_ssh_key "localhost" "$DOCKER_SSH_PORT"

        # Copy the SSH key to all targets
        # Test SSH connections and accept new keys automatically
        echo "ROVER COMPUTER PASSWORD"
        $ROVER_CONNECT -o StrictHostKeyChecking=accept-new marsrover@$ROVER_IP_ADDRESS "echo" &> /dev/null
        ssh-copy-id marsrover@$ROVER_IP_ADDRESS

        echo "ROVER DOCKER COMPUTER PASSWORD"
        $DOCKER_CONNECT -o StrictHostKeyChecking=accept-new marsrover@$ROVER_IP_ADDRESS -p $DOCKER_SSH_PORT "echo" &> /dev/null
        ssh-copy-id -p $DOCKER_SSH_PORT marsrover@$ROVER_IP_ADDRESS

        echo "THIS COMPUTER DOCKER PASSWORD"
        ssh-copy-id -p $DOCKER_SSH_PORT marsrover@localhost
        $BASE_CONNECT -o StrictHostKeyChecking=accept-new marsrover@localhost -p $DOCKER_SSH_PORT "echo" &> /dev/null

        ;;
    *)
        # If the user enters an invalid option, exit with an error message
        echo "Invalid choice. Exiting."
        exit 1
        ;;
esac

