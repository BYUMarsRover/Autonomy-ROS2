#!/bin/bash

#TODO add comments and error handling


echo "Do you want to send the key to the rover, the rover docker container, or both?"
echo "1) Rover"
echo "2) Rover Docker Container"
echo "3) This Computer (ex. Base Station) Docker Conatiner"
echo "4) All"
read -p "Enter your choice (1/2/3): " choice

ROVER_IP_ADDRESS=192.168.1.120
DOCKER_SSH_PORT=2233

case $choice in
    1)
        ssh-copy-id marsrover@$ROVER_IP_ADDRESS
        ssh -o StrictHostKeyChecking=accept-new marsrover@$ROVER_IP_ADDRESS "echo" &> /dev/null
        ;;
    2)
        ssh-copy-id -p $DOCKER_SSH_PORT marsrover@$ROVER_IP_ADDRESS
        ssh -o StrictHostKeyChecking=accept-new marsrover@$ROVER_IP_ADDRESS -p $DOCKER_SSH_PORT "echo" &> /dev/null
        ;;
    3)
        ssh-copy-id -p $DOCKER_SSH_PORT marsrover@localhost
        ssh -o StrictHostKeyChecking=accept-new marsrover@localhost -p $DOCKER_SSH_PORT "echo" &> /dev/null
        ;;
    4)
        ssh-copy-id marsrover@$ROVER_IP_ADDRESS
        ssh-copy-id -p $DOCKER_SSH_PORT marsrover@$ROVER_IP_ADDRESS
        ssh-copy-id -p $DOCKER_SSH_PORT marsrover@localhost

        ssh -o StrictHostKeyChecking=accept-new marsrover@localhost -p $DOCKER_SSH_PORT "echo" &> /dev/null
        ssh -o StrictHostKeyChecking=accept-new marsrover@$ROVER_IP_ADDRESS -p $DOCKER_SSH_PORT "echo" &> /dev/null
        ssh -o StrictHostKeyChecking=accept-new marsrover@$ROVER_IP_ADDRESS "echo" &> /dev/null

        ;;
    *)
        echo "Invalid choice. Exiting."
        exit 1
        ;;
esac

echo "Key(s) copied successfully."


