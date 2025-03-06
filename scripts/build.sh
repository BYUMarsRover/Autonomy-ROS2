#!/bin/bash

echo "This script will build the Autonomy-ROS2 project."
echo "You have the option to perform a clean build, which will remove the build, install, and log directories before building."

read -p "Do you want to perform a clean build? (y/n): " clean_build

if [ "$clean_build" == "y" ]; then
    echo "Performing a clean build..."
    docker exec -it autonomy_ros2 bash -c "rm -rf /home/marsrover/mars_ws/build /home/marsrover/mars_ws/install /home/marsrover/mars_ws/log"
fi

COMMAND="source /opt/ros/humble/setup.bash && cd /home/marsrover/mars_ws && colcon build"

echo "Building the project..."
docker exec -it autonomy_ros2 bash -c "$COMMAND"

echo "Build process completed."