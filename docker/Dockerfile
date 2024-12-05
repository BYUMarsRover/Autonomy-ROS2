# Created by Nelson Durrant, Oct 2024
FROM ros:humble-ros-base

# Update and upgrade
RUN apt update && apt upgrade -y

# Install dependencies needed below
RUN apt update && apt install -y \
    sudo \
    curl \
    libx11-dev \
    libxext-dev \
    libxrender-dev \
    libxrandr-dev \
    qtbase5-dev \
    libqt5x11extras5 \
    python3-gi \
    gir1.2-gtk-3.0 \
    x11-apps \
    ros-humble-rqt* \
    vim \
    docker.io \
    ros-humble-robot-localization \
    python3-pip

# Set up a new user
RUN useradd -ms /bin/bash marsrover
RUN usermod -aG sudo marsrover
RUN usermod -aG dialout marsrover
RUN echo 'marsrover:marsrover' | chpasswd
USER marsrover
WORKDIR /home/marsrover

# Set up automatic sourcing
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN /bin/bash -c "source ~/.bashrc"


# Quick permissions fix (might be too heavy-handed)
USER root
RUN chown -R marsrover: /usr/local
USER marsrover

# Add OpenCV dependency -- needed for aruco detect
USER root
RUN apt-get update && \
apt-get install -y libopencv-dev && \
rm -rf /var/lib/apt/lists/*

# Set CMake prefix path for OpenCV and ROS
# ENV CMAKE_PREFIX_PATH="/usr/local/share/OpenCV;/opt/ros/humble"
USER marsrover

# Install ROS2 dependencies for aruco_detect
# USER root
# RUN apt update && \
#     apt install -y ros-humble-tf2-geometry-msgs ros-humble-visualization-msgs && \
#     rm -rf /var/lib/apt/lists/*
# USER marsrover

# "transforms3d" uses a function depreciated in numpy 2.0
RUN pip3 install numpy==1.26.0
# Install pydantic version 1.10.9; used by usb_cam launch file
RUN pip3 install pydantic==1.10.9

USER root
RUN apt update && apt install -y \
    libboost-dev \
    ros-humble-rviz-common
# Install USB Camera Package
RUN apt update && apt install -y ros-humble-usb-cam
# Tmux used for rinning the pipline
RUN apt update && apt install -y tmux
# Install mapviz and related packages
RUN apt update && apt install -y \
    ros-humble-mapviz \
    ros-humble-mapviz-plugins \
    ros-humble-tile-map \
    ros-humble-multires-image \
    ros-humble-rosapi
USER marsrover

# Edit so it tries to build (YOU CAN REMOVE THIS)