#!/bin/bash
# Created by Nelson Durrant, Jan 2025
#
# Runs the domain bridge
#
# IMPORTANT: I set up this bridge in an attempt to limit communication between nodes on the rover
# and base station due to antenna bandwidth constraints. IT DID NOT WORK FOR THAT. We actually
# noted a large increase in bandwidth usage when running the bridge. I'm leaving the code here
# for reference in the future, but it is not recommended for use in the current project.
# - Nelson Durrant, Jan 2025

source ~/mars_ws/install/setup.bash
ros2 run domain_bridge domain_bridge ~/mars_ws/domain_bridge/mars_bridge_config.yaml
