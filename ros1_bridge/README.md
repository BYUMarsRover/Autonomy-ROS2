## ROS 1 Bridge Instructions

This guide outlines the steps to set up and run the containerized ROS 1 bridge.

**Preparation:**

- Add any custom ROS 1 and ROS 2 message packages to the `ros1_msgs_ws` and `ros2_msgs_ws` workspaces.

- Include any necessary mapping declarations for those custom messages in `ros2_msgs_ws/src/ros2_bridge_mappings/mapping_rules.yaml`. Refer to the official documentation for help: [https://github.com/ros2/ros1_bridge/blob/master/doc/index.rst](https://github.com/ros2/ros1_bridge/blob/master/doc/index.rst)

- Edit the `TEST_MSG_TYPE` and `TEST_MSG_DATA` variables at the beginning of `test_bridge.sh` to match the desired custom message type.

**Building and Testing the Bridge:**

- Run `bash compose.sh` to launch and enter the Docker container.

- Run `bash build_bridge.sh` inside the container to build the bridge.

- Run `bash test_bridge.sh` inside the container to verify the bridge works.
  
**Running the Bridge:**

- Run `bash compose.sh` to launch and enter the Docker container.

- Start ROS 1 and ROS 2 instances on the host network.

- Run `bash run_bridge.sh` inside the container to run the bridge.

--

Created by Nelson Durrant, Oct 2024.
