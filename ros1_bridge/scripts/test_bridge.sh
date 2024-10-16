#!/bin/bash
# Created by Nelson Durrant, Oct 2024
#
# Tests the ROS1 bridge using the included ROS1 and ROS2 msg packages

# Define the message type and data to be tested
export TEST_MSG_TYPE="rover_msgs/msg/RoverStateSingleton"
export TEST_MSG_DATA="map_roll: 1.0"

function printInfo {
  echo -e "\033[0m\033[36m[INFO] $1\033[0m"
}

function printWarning {
  echo -e "\033[0m\033[33m[WARNING] $1\033[0m"
}

function printError {
  echo -e "\033[0m\033[31m[ERROR] $1\033[0m"
}

# Create a new tmux session
tmux new-session -d -s bridge_session

# Create new panes
tmux split-window -h -t bridge_session
tmux split-window -v -t bridge_session
tmux split-window -v -t bridge_session
tmux select-pane -t bridge_session:0.0

# Run the ROS1 core
tmux send-keys -t bridge_session:0.3 "source /opt/ros/noetic/setup.bash" ENTER
tmux send-keys -t bridge_session:0.3 "source ~/ros1_msgs_ws/install_isolated/setup.bash" ENTER
tmux send-keys -t bridge_session:0.3 "roscore" ENTER

# Start publishing from ROS2
tmux send-keys -t bridge_session:0.2 "source ~/ros2_humble/install/setup.bash" ENTER
tmux send-keys -t bridge_session:0.2 "source ~/ros2_msgs_ws/install/local_setup.bash" ENTER
tmux send-keys -t bridge_session:0.2 "ros2 topic pub /test $TEST_MSG_TYPE '$TEST_MSG_DATA'" ENTER

# Start the bridge
tmux send-keys -t bridge_session:0.1 "source /opt/ros/noetic/setup.bash" ENTER
tmux send-keys -t bridge_session:0.1 "source ~/ros1_msgs_ws/install_isolated/setup.bash" ENTER
tmux send-keys -t bridge_session:0.1 "source ~/ros2_humble/install/setup.bash" ENTER
tmux send-keys -t bridge_session:0.1 "source ~/ros2_msgs_ws/install/local_setup.bash" ENTER
tmux send-keys -t bridge_session:0.1 "source ~/bridge_ws/install/local_setup.bash" ENTER
tmux send-keys -t bridge_session:0.1 "ros2 run ros1_bridge dynamic_bridge --bridge-all-topics" ENTER

echo ""
printInfo "Wait just a second while we get the ROS1 bridge test set up..."
printInfo "You can detach from the tmux session by pressing Ctrl+B, then D."
echo ""

sleep 3

# Start listening to the ROS1 topic
tmux send-keys -t bridge_session:0.0 "source /opt/ros/noetic/setup.bash" ENTER
tmux send-keys -t bridge_session:0.0 "source ~/ros1_msgs_ws/install_isolated/setup.bash" ENTER
tmux send-keys -t bridge_session:0.0 "rostopic echo -n 1 /test" ENTER

# Attach to the tmux session so we can see the output
tmux attach -t bridge_session && tmux kill-session -t bridge_session