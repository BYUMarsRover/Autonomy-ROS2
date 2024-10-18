#!/bin/bash
# Created by Nelson Durrant, Oct 2024
#
# Builds the ROS1 bridge using the included ROS1 and ROS2 msg packages

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
tmux select-pane -t bridge_session:0.0

# Build ROS1 messages
tmux send-keys -t bridge_session:0.1 "source /opt/ros/noetic/setup.bash" ENTER
tmux send-keys -t bridge_session:0.1 "cd ~/ros1_msgs_ws" ENTER
tmux send-keys -t bridge_session:0.1 "catkin_make_isolated --install" ENTER

# Build ROS2 messages
tmux send-keys -t bridge_session:0.2 "source ~/ros2_humble/install/setup.bash" ENTER
tmux send-keys -t bridge_session:0.2 "cd ~/ros2_msgs_ws" ENTER
tmux send-keys -t bridge_session:0.2 "colcon build" ENTER

echo ""
printInfo "Wait just a second while we build the included ROS1 and ROS2 msg packages..."
printInfo "You can detach from the tmux session by pressing Ctrl+B, then D."
echo ""

# Wait until the ROS1 and ROS2 messages finish building
sleep 3
while pgrep -u $UID -f "catkin_make_isolated" > /dev/null || pgrep -u $UID -f "colcon build" > /dev/null; do
    sleep 1
done

# Source ROS1 and ROS2 environments and build the ROS1 bridge
tmux send-keys -t bridge_session:0.0 "source /opt/ros/noetic/setup.bash" ENTER
tmux send-keys -t bridge_session:0.0 "source ~/ros1_msgs_ws/install_isolated/setup.bash" ENTER
tmux send-keys -t bridge_session:0.0 "source ~/ros2_humble/install/setup.bash" ENTER
tmux send-keys -t bridge_session:0.0 "source ~/ros2_msgs_ws/install/local_setup.bash" ENTER
tmux send-keys -t bridge_session:0.0 "cd ~/bridge_ws" ENTER
tmux send-keys -t bridge_session:0.0 "colcon build --packages-select ros1_bridge --cmake-force-configure" ENTER

# Attach to the tmux session so we can see the output
tmux attach -t bridge_session && tmux kill-session -t bridge_session
