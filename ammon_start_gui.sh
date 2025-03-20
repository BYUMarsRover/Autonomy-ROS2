#!/bin/bash
# Created by Ammon Wolfert, Feb 2025
#
# Start the science debug gui

colcon build --packages-select science
source install/setup.bash
ros2 run science science_debug