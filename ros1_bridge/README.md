Created by Nelson Durrant, Oct 2024

To set up and run the ROS 1 bridge, simply:
(1) Add any custom ROS 1 and ROS 2 msg packages to 'ros1_msgs_ws' and 'ros2_msgs_ws'
(2) Add any necessary mapping declarations for those custom msg packages to 'ros2_msgs_ws/src/ros2_bridge_mappings/mapping_rules.yaml' (see https://github.com/ros2/ros1_bridge/blob/master/doc/index.rst)
(3) Edit the 'TEST_MSG_TYPE' and 'TEST_MSG_DATA' declarations at the top of 'test_bridge.sh' to match the desired custom message type
(4) Run 'bash compose.sh' to start and enter the Docker container
(5) Run 'bash build_bridge.sh' inside the container to build the bridge
(6) Run 'bash test_bridge.sh' inside the container to test the bridge
(7) Start ROS 1 and ROS 2 instances on the host network and run 'bash run_bridge.sh' inside the container to run the bridge