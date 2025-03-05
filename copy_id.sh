#!/bin/bash

echo "Do you want to send the key to the rover, the rover docker container, or both?"
echo "1) Rover"
echo "2) Rover Docker Container"
echo "3) Both"
read -p "Enter your choice (1/2/3): " choice

case $choice in
    1)
        ssh-copy-id marsrover@192.168.1.120
        ;;
    2)
        ssh-copy-id -p 2233 marsrover@192.168.1.120
        ;;
    3)
        ssh-copy-id marsrover@192.168.1.120
        ssh-copy-id -p 2233 marsrover@192.168.1.120
        ;;
    *)
        echo "Invalid choice. Exiting."
        exit 1
        ;;
esac

echo "Key(s) copied successfully."