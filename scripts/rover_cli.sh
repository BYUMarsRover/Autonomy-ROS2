IP_ADDRESS="127.0.0.1"
export FASTRTPS_DEFAULT_PROFILES_FILE="/home/marsrover/mars_ws/super_rover_client_configuration_file.xml"

# Set up the ROS 2 CLI tools with Fast DDS
export ROS_DISCOVERY_SERVER="${IP_ADDRESS}:11811"

# Restart the ROS 2 daemon
ros2 daemon stop
ros2 daemon start

echo "ROS 2 CLI tools have been set up with Fast DDS using IP address ${IP_ADDRESS}."