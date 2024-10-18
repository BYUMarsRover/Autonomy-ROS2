#!/bin/bash
# Created by Nelson Durrant, Oct 2024
#
# Starts and stops the ROS1 bridge image

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
    echo ""
    printInfo "Stopping the ROS1 bridge image..."
    echo ""

    docker compose -f docker/docker-compose.yaml down
    ;;
  *)
    echo ""
    printInfo "Loading the ROS1 bridge image..."
    echo ""

    docker compose -f docker/docker-compose.yaml up -d
    docker exec -it ros1_bridge bash
    ;;
esac
