#!/bin/bash
# Created by Ammon Wolfert, Feb 2025
#
# Open and source the docker container

echo "Entering container and sourcing..."
xhost +local:docker
docker exec -it autonomy_ros2 bash -c "
    cd /home/marsrover/mars_ws
    source install/setup.bash
    echo 'Environment sourced. You are now inside the Docker container.'
    exec bash  # Start an interactive shell and stay inside the container
"
