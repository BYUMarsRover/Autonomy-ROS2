# Created by Nelson Durrant, Oct 2024
FROM ros:humble-ros-base 

# Update and upgrade
RUN apt update && apt upgrade -y

# Install general dependencies
RUN apt update && apt install -y \
    sudo \
    curl \
    vim \
    tmux \
    mosh \
    docker.io \
    python3-pip \
    libboost-dev

# Install graphical interface dependencies
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

# Set up automatic ROS 2 sourcing and colorized output
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN export "RCUTILS_COLORIZED_OUTPUT=1" >> ~/.bashrc

# Quick permissions fix (might be too heavy-handed)
USER root
RUN chown -R marsrover: /usr/local
USER marsrover

##############################################################################
# IMPORTANT! When adding dependencies or commands below this point in the
# Dockerfile, please document your changes following this format:
#     
#     # What do these commands do? - Name, Month Year
#     # Why do we need them? Which ROS 2 nodes require them?
#     < Add Dockerfile commands here >
##############################################################################

RUN pip3 install utm

# Add OpenCV dependency -- needed for aruco detect
USER root
RUN apt update && apt install -y libopencv-dev 
USER marsrover

# "transforms3d" uses a function depreciated in numpy 2.0
RUN pip3 uninstall -y numpy
RUN pip3 install numpy==1.26.0

# Install pydantic version 1.10.9; used by usb_cam launch file
RUN pip3 install pydantic==1.10.9

RUN pip3 install odpslides
RUN pip3 install pyserial

RUN pip3 install roboticstoolbox-python

USER root
RUN apt update && apt install -y \
    ros-humble-rviz-common

# Install USB Camera Package
RUN apt update && apt install -y ros-humble-usb-cam

# Tmux used for rinning the pipline
RUN apt update && apt install -y \
    ros-humble-robot-localization \
    ros-humble-tf-transformations

# Install mapviz and related packages
USER root
RUN apt update && apt install -y \
    ros-humble-mapviz \
    ros-humble-mapviz-plugins \
    ros-humble-tile-map \
    ros-humble-multires-image \
    ros-humble-rosapi
USER marsrover

# Install domain bridge package - Nelson Durrant, Jan 2025
# Necessary dependency for running the domain bridge scripts
USER root
RUN apt update && apt install -y ros-humble-domain-bridge
USER marsrover

# Install the joy package - Alyssa Fielding, Feb 2025
# Needed for the joystick control on the base station
USER root
RUN apt update && apt install -y ros-humble-joy
USER marsrover
