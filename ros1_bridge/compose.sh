#!/bin/bash
# Created by Nelson Durrant, Oct 2024
#
# Starts and stops the ROS 1 bridge image

# Match this username to the one defined in the Dockerfile
export UNAME=marsrover

function printInfo {
  echo -e "\033[0m\033[36m[INFO] $1\033[0m"
}

function printWarning {
  echo -e "\033[0m\033[33m[WARNING] $1\033[0m"
}

function printError {
  echo -e "\033[0m\033[31m[ERROR] $1\033[0m"
}

case $1 in
  "down")
    printInfo "Stopping the ROS 1 bridge image..."
    docker compose -f docker/docker-compose.yaml down
    ;;
  *)
    printInfo "Loading the ROS 1 bridge image..."
    docker compose -f docker/docker-compose.yaml up -d
    docker exec -it ros1_bridge bash
    ;;
esac
