# Created by Nelson Durrant, Oct 2024

# Start from the ROS 2 Humble base image 
FROM ros:humble-ros-base

# Get the architecture of the target machine
ARG TARGETARCH

# Update and upgrade
RUN apt update && apt upgrade -y

# Install general system dependencies
RUN apt update && apt install -y \
    sudo \
    curl \
    vim \
    nano \
    docker.io \
    python3-pip \
    tmux \
    mosh

# Install GUI tools
RUN apt update && apt install -y \
    libx11-dev \
    libxext-dev \
    libxrender-dev \
    libxrandr-dev \
    qtbase5-dev \
    libqt5x11extras5 \
    python3-gi \
    gir1.2-gtk-3.0 \
    x11-apps \
    ros-humble-rqt*

# Set up a new user
RUN useradd -ms /bin/bash marsrover
RUN usermod -aG sudo marsrover
RUN usermod -aG dialout marsrover
RUN echo 'marsrover:marsrover' | chpasswd
USER marsrover
WORKDIR /home/marsrover

# Set up automatic ROS environment sourcing
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Colorized ROS 2 logging output
RUN echo "export RCUTILS_COLORIZED_OUTPUT=1" >> ~/.bashrc

# Quick permissions fix (might be too heavy-handed)
USER root
RUN chown -R marsrover: /usr/local
USER marsrover

####################################################################################################
# IMPORTANT! To document the changes made to this file after this point, please add a comment
# above each line you add following this format:
#
#   # Quick Description - Your name, Month Year
#   # Why are you adding this? What nodes/functionality is it needed for?
#   <Dockerfile code here>
# 
####################################################################################################

# Install numpy 1.26.0 - ADD HERE
# "transforms3d" uses a function depreciated in numpy 2.0
RUN pip3 uninstall -y numpy
RUN pip3 install numpy==1.26.0

# Install utm - ADD HERE
# ADD HERE
RUN pip3 install utm

# Install localization and transform packages - ADD HERE
# ADD HERE
USER root
RUN apt update && apt install -y \
    ros-humble-robot-localization \
    ros-humble-tf-transformations
USER marsrover

# Add OpenCV dependency - ADD HERE
# Needed for aruco detect node
USER root
RUN apt update && apt install -y libopencv-dev
# ENV CMAKE_PREFIX_PATH="/usr/local/share/OpenCV;/opt/ros/humble"
USER marsrover

# TO DO: DO WE NEED THIS?
# Install ROS2 dependencies for aruco_detect
# USER root
# RUN apt update && \
#     apt install -y ros-humble-tf2-geometry-msgs ros-humble-visualization-msgs && \
#     rm -rf /var/lib/apt/lists/*
# USER marsrover

# Install pydantic version 1.10.9 - ADD HERE
# This version is used by the usb_cam launch file
RUN pip3 install pydantic==1.10.9
RUN pip3 install odpslides
RUN pip3 install pyserial

# Install USB camera package - ADD HERE
# ADD HERE
RUN apt update && apt install -y ros-humble-usb-cam

# Install roboticstoolbox-python - ADD HERE
# ADD HERE
RUN pip3 install roboticstoolbox-python

# Install ROS 2 dependencies for mapviz - ADD HERE
# ADD HERE
USER root
RUN apt update && apt install -y \
    libboost-dev \
    ros-humble-rviz-common

# Install mapviz and related packages
USER root
RUN if [ "$TARGETARCH" = "amd64" ]; then \
    apt update && apt install -y \
    ros-humble-mapviz \
    ros-humble-mapviz-plugins \
    ros-humble-tile-map \
    ros-humble-multires-image \
    ros-humble-rosapi; \
    fi
USER marsrover

# Domain bridge setup - Nelson Durrant, Jan 2025
# Needed to run a domain bridge between two different ROS 2 domains
USER root
RUN apt update && apt install -y ros-humble-domain-bridge
USER marsrover
