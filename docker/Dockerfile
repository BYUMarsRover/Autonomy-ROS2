# Created by Nelson Durrant, Oct 2024
FROM ros:humble-ros-base
ARG DEBIAN_FRONTEND=noninteractive

# Define a username for the new user
ARG NAME=marsrover

# Update and upgrade
RUN apt update && apt upgrade -y

# Set up the direct Docker ssh connection
RUN sudo apt update && apt install -y openssh-server
RUN mkdir -p /var/run/sshd

RUN sed -i 's/#PasswordAuthentication yes/PasswordAuthentication yes/' /etc/ssh/sshd_config
RUN sed -i 's/PasswordAuthentication no/PasswordAuthentication yes/' /etc/ssh/sshd_config
RUN sed -i 's/\(^Port\)/#\1/' /etc/ssh/sshd_config && echo Port 2233 >> /etc/ssh/sshd_config

EXPOSE 2233

# Install general dependencies
RUN apt update && apt install -y \
    sudo \
    curl \
    vim \
    tmux \
    docker.io \
    python3-pip \
    libboost-dev

# Install graphical interface dependencies
RUN apt update && apt install -y \
    x11-apps \
    x11-utils \
    x11-xserver-utils \
    xauth \
    ros-humble-rqt*

# Set up a new user
RUN useradd -ms /bin/bash $NAME
RUN usermod -aG sudo $NAME
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
RUN usermod -aG video $NAME
RUN usermod -aG dialout $NAME
RUN echo "$NAME:$NAME" | chpasswd
USER $NAME
WORKDIR /home/$NAME

# Set up automatic ROS 2 sourcing and colorized output
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /home/$NAME/mars_ws/install/setup.bash" >> ~/.bashrc
RUN echo "export RCUTILS_COLORIZED_OUTPUT=1" >> ~/.bashrc
RUN touch ~/.hushlogin

# Quick permissions fix (might be too heavy-handed)
USER root
RUN chown -R $NAME: /usr/local
USER $NAME

###############################################################
# IMPORTANT! When adding dependencies or Dockerfile commands
# below, please document your changes following this format:
#     
#     # What do these commands do? - Name, Month Year
#     # Why do we need them? Which ROS 2 nodes require them?
#     < Add Dockerfile commands here >
#
###############################################################

# Install GPS to UTM convertor - Daniel Webb, Feb 2025
# Needed for path planning and state machine logic
RUN pip3 install utm

# Install OpenCV - Gabe Slade, Feb 2025
# Used for detecting and locating aruco tags
USER root
RUN apt update && apt install -y libopencv-dev 
USER $NAME

# Downgrade numpy - Nelson Durrant, Feb 2025
# "transforms3d" (quaternion conversion) uses a function depreciated in numpy 2.0
RUN pip3 uninstall -y numpy
RUN pip3 install numpy==1.26.0

# Install pydantic 1.10.9 - Daniel Webb, Feb 2025
# Necessary dependency for usb_cam launch file
RUN pip3 install pydantic==1.10.9

# Install pyserial - Braden Meyers, Feb 2025
# Needed for parsing serial communication from the mega
RUN pip3 install pyserial

# Install rviz - Gabe Slade, Feb 2025
# Needed in order to display various system GUIs (including rover orientation + hazards)
USER root
RUN apt update && apt install -y ros-humble-rviz-common
RUN apt update && apt install -y ros-humble-rviz2
USER $NAME

# Install ROS 2 usb-cam package - Daniel Webb, Feb 2025
# Needed to interface with the camera for aruco tag detection
USER root
RUN apt update && apt install -y ros-humble-usb-cam
USER $NAME

# Install ROS 2 SLAM packages - Braden Meyers, Feb 2025
# Needed to fuse the magnetometer and visual odom data
USER root
RUN apt update && apt install -y \
    ros-humble-robot-localization \
    ros-humble-tf-transformations
USER $NAME

# Install mapviz - Sarah Sanderson, Feb 2025
# Necessary packages to run the mapviz viewer
# USER root
# RUN apt update && apt install -y \
#     ros-humble-mapviz \
#     ros-humble-mapviz-plugins \
#     ros-humble-tile-map \
#     ros-humble-multires-image \
#     ros-humble-rosapi
# USER $NAME

# Install domain bridge package - Nelson Durrant, Feb 2025
# Necessary dependency for running the domain bridge scripts
USER root
RUN apt update && apt install -y ros-humble-domain-bridge
USER $NAME

# Install the joy package - Alyssa Fielding, Feb 2025
# Needed for the joystick control on the base station
USER root
RUN apt update && apt install -y ros-humble-joy
USER $NAME 

# Install gpsd-client - Braden Meyers, Feb 2025
# Needed to publish raw gps data if we use gpsd
USER root
RUN apt update && apt install -y gpsd-clients
USER $NAME

# Install pcl and open3D - Gabe Slader Mar 2025
# Needed for the hazard avoidance node 
RUN pip3 install open3d
RUN pip3 install pcl 

# Install astar for path planning
RUN pip3 install astar rasterio

# Install libboost for ublox read - Braden Meyers, Mar 2025
# Needed to run UBLOX_read_2
USER root
RUN apt update && apt install -y libboost-all-dev
USER $NAME

# Install urdf launch - Braden Meyers, Apr 2025
# Needed to publish the urdf model of the rover
# ZED messages for the the object detections
USER root
RUN apt update && apt install -y \
    ros-humble-urdf-launch \
    ros-humble-zed-msgs    
USER $NAME


# Install tools for terrain maps - Nelson Durrant, Apr 2025
# Used for loading in and parsing GeoTIFF elevation maps
USER root
RUN apt update && apt install -y \
    libgdal-dev \
    python3-rasterio
USER $NAME

# Install AStar planning package - Nelson Durrant, Apr 2025
# Simple A* path planning algorithm implementation for 2D maps
RUN pip3 install astar