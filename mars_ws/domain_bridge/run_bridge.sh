#!/bin/bash
# Created by Nelson Durrant, Jan 2025
#
# Runs the domain bridge

source ~/mars_ws/install/setup.bash
ros2 run domain_bridge domain_bridge ~/mars_ws/domain_bridge/mars_bridge_config.yaml
