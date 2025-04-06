#!/bin/bash

# create symlinks to the rules in our repo
echo "symlink udev rules to  /etc/udev/rules.d/"
sudo ln -s ~/Autonomy-ROS2/scripts/udev/rules/*.rules /etc/udev/rules.d
printf "\nReloading udev\n\n"
sudo ~/Autonomy-ROS2/scripts/udev/reload.sh