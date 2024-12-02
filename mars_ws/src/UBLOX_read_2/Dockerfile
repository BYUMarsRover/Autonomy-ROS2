# Base image
ARG ROS_DISTRO
FROM ros:${ROS_DISTRO}

# Update system
RUN apt update
RUN apt upgrade -y
RUN rosdep update

# Install utilities/packages
RUN apt install\
    ros-dev-tools\
    vim\
    -y

# Set working directory
WORKDIR /ublox_read_2

# Install ros package dependencies
COPY . .
RUN rosdep install --from-paths . --ignore-src -y

# Build packages
# If mounting src files in compose.yaml, don't include build in Dockerfile to avoid confusion
#  with potentially desynced build files and src files.
#RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build"

# Setup ROS environment variables
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc
RUN echo "source /ublox_read_2/install/setup.bash" >> /root/.bashrc
