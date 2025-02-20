#!/bin/bash
# Created by Nelson Durrant, Oct 2024
#
# Starts and stops the Autonomy ROS2 image

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
    printInfo "Stopping the Autonomy ROS2 image..."
    docker compose -f docker/docker-compose.yaml down
    ;;
  "build")
    printInfo "Loading the Autonomy ROS2 image..."
    docker compose -f docker/docker-compose.yaml up -d
    docker exec -it autonomy_ros2 bash
    ;;
  *)
    printInfo "Entering Container... (run 'bash compose.sh build' to build or create the container)"
    docker exec -it autonomy_ros2 bash
    ;;
esac