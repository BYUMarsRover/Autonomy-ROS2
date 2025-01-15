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
    ros-humble-robot-localization

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

# need to rollpack on numpy to v1.26.0 to avoid bug with the odometry package (transforms3d dependency)
# Need to confirm this is still necessary?
# RUN pip3 uninstall -y numpy
# RUN pip3 install numpy==1.26.0

RUN echo "export RCUTILS_COLORIZED_OUTPUT=1" >> ~/.bashrc
