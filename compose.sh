#!/bin/bash
# Created by Nelson Durrant, Oct 2024
#
# Manages our Docker containers and the 'rover_dev' tmux session

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

case $1 in
  "down")
    printWarning "Stopping the Autonomy ROS2 container..."
    docker compose -f docker/docker-compose.yaml down
    ;;
  *)
    printInfo "Loading the Autonomy ROS2 container..."
    docker compose -f docker/docker-compose.yaml up -d

    # Skip 'rover_dev' tmux session creation on the rover
    if [ ! "$(uname -m)" == "aarch64" ]; then

      # Check if the 'rover_dev' tmux session exists
      if [ "$(docker exec -it autonomy_ros2 tmux list-sessions | grep rover_dev)" == "" ]; then
        # If not, create a new 'rover_dev' tmux session
        printWarning "Creating a new tmux session..."
        docker exec -it autonomy_ros2 tmux new-session -d -s rover_dev
        docker exec -it autonomy_ros2 tmux select-pane -t 0 -T rover_dev
        docker exec -it autonomy_ros2 tmux send-keys "clear && cat ~/scripts/introduction.txt" Enter
      fi
      # Attach to the 'rover_dev' tmux session
      docker exec -it autonomy_ros2 tmux attach -t rover_dev
    fi
    ;;
esac
