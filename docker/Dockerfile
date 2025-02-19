# Created by Nelson Durrant, Oct 2024
FROM ros:humble-ros-base
ARG DEBIAN_FRONTEND=noninteractive

# Update and upgrade
RUN apt update && apt upgrade -y

# Set up the direct Docker ssh connection
RUN sudo apt update && apt install -y openssh-server
RUN mkdir -p /var/run/sshd

# Set up a new user
RUN useradd -ms /bin/bash marsrover
RUN usermod -aG sudo marsrover
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
RUN usermod -aG video marsrover
RUN usermod -aG dialout marsrover
RUN echo 'marsrover:marsrover' | chpasswd
# USER marsrover
# WORKDIR /home/marsrover

# Docker ssh configuration
RUN sed -i 's/#PasswordAuthentication yes/PasswordAuthentication yes/' /etc/ssh/sshd_config
RUN sed -i 's/PasswordAuthentication no/PasswordAuthentication yes/' /etc/ssh/sshd_config
RUN sed -i 's/\(^Port\)/#\1/' /etc/ssh/sshd_config && echo Port 2233 >> /etc/ssh/sshd_config

EXPOSE 2233

# Install graphical interface dependencies
RUN apt update && apt install -y \
    x11-apps \
    x11-utils \
    x11-xserver-utils \
    xauth \
    ros-humble-rqt*

# Install general dependencies
RUN apt update && apt install -y \
    sudo \
    curl \
    vim \
    tmux \
    docker.io \
    python3-pip \
    libboost-dev

# Set up automatic ROS 2 sourcing and colorized output
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /home/marsrover/mars_ws/install/setup.bash" >> ~/.bashrc
RUN echo "export RCUTILS_COLORIZED_OUTPUT=1" >> ~/.bashrc

# Enter tmux session on Docker entry
RUN echo "tmux attach-session -t autonomy_ros2" >> ~/.bashrc

# Quick permissions fix (might be too heavy-handed)
# USER root
# RUN chown -R marsrover: /usr/local
# USER marsrover

###############################################################
# IMPORTANT! When adding dependencies or Dockerfile commands
# below, please document your changes following this format:
#     
#     # What do these commands do? - Name, Month Year
#     # Why do we need them? Which ROS 2 nodes require them?
#     < Add Dockerfile commands here >
#
###############################################################

# # Install GPS to UTM convertor - Daniel Webb, Feb 2025
# # Needed for path planning and state machine logic
# RUN pip3 install utm

# # Install OpenCV - Gabe Slade, Feb 2025
# # Used for detecting and locating aruco tags
# USER root
# RUN apt update && apt install -y libopencv-dev 
# USER marsrover

# # Downgrade numpy - Nelson Durrant, Feb 2025
# # "transforms3d" (quaternion conversion) uses a function depreciated in numpy 2.0
# RUN pip3 uninstall -y numpy
# RUN pip3 install numpy==1.26.0

# # Install pydantic 1.10.9 - Daniel Webb, Feb 2025
# # Necessary dependency for usb_cam launch file
# RUN pip3 install pydantic==1.10.9

# # Install pyserial - Braden Meyers, Feb 2025
# # Needed for parsing serial communication from the mega
# RUN pip3 install pyserial

# # Install rviz - Gabe Slade, Feb 2025
# # Needed in order to display various system GUIs
# USER root
# RUN apt update && apt install -y \
#     ros-humble-rviz-common
# USER marsrover

# # Install ROS 2 usb-cam package - Daniel Webb, Feb 2025
# # Needed to interface with the camera for aruco tag detection
# USER root
# RUN apt update && apt install -y \
#     ros-humble-usb-cam
# USER marsrover

# # Install ROS 2 SLAM packages - Braden Meyers, Feb 2025
# # Needed to fuse the magnetometer and visual odom data
# USER root
# RUN apt update && apt install -y \
#     ros-humble-robot-localization \
#     ros-humble-tf-transformations
# USER marsrover

# # Install mapviz - Sarah Sanderson, Feb 2025
# # Necessary packages to run the mapviz viewer
# # USER root
# # RUN apt update && apt install -y \
# #     ros-humble-mapviz \
# #     ros-humble-mapviz-plugins \
# #     ros-humble-tile-map \
# #     ros-humble-multires-image \
# #     ros-humble-rosapi
# # USER marsrover

# # Install domain bridge package - Nelson Durrant, Feb 2025
# # Necessary dependency for running the domain bridge scripts
# USER root
# RUN apt update && apt install -y ros-humble-domain-bridge
# USER marsrover

# # Install the joy package - Alyssa Fielding, Feb 2025
# # Needed for the joystick control on the base station
# USER root
# RUN apt update && apt install -y ros-humble-joy
# USER marsrover 
