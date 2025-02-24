## Simple ROS 1 Bridge Guide

This guide outlines the steps to set up and run a containerized bridge between ROS 1 Noetic and ROS 2 Humble. It allows for the inclusion of custom message types.

--

### Preparation:

- Add any custom ROS 1 and ROS 2 message packages to the `ros1_msgs_ws` and `ros2_msgs_ws` workspaces.

- Include any necessary mapping declarations for those custom messages in `ros2_msgs_ws/src/ros2_bridge_mappings/mapping_rules.yaml`. Refer to the official documentation for help: [https://github.com/ros2/ros1_bridge/blob/master/doc/index.rst](https://github.com/ros2/ros1_bridge/blob/master/doc/index.rst)

- Edit the `TEST_MSG_TYPE` and `TEST_MSG_DATA` variables at the beginning of `test_bridge.sh` to match the desired custom message type.

- Edit the `ROS_MASTER_URI` environment variable in `run_bridge.sh` to match the ROS 1 master configuration.

--

### Building and Testing the Bridge:

- Run `bash compose.sh` to launch and enter the Docker container. (Only the arm64 image is currently pushed to Docker Hub. To run it on amd64 architecture, you will need to build the image locally.)

- Run `bash build_bridge.sh` inside the container to build the bridge. (This will take a long time to run after making changes. Like 2-3 hours.)

- Run `bash test_bridge.sh` inside the container to verify the bridge works.

--
  
### Running the Bridge:

- Start ROS 1 and ROS 2 instances on the host network.

- Run `bash compose.sh` to launch and enter the Docker container.

- Run `bash run_bridge.sh` inside the container to run the bridge.

> **NOTE:** The ROS 1 bridge might throw the error:
> 
> "failed to create 2to1 bridge for topic '/rosout' with ROS 2 type 'rcl_interfaces/msg/Log' and ROS 1 type 'rosgraph_msgs/Log': No template specialization for the pair"
> 
> This error is a known issue with the larger open-source project and can safely be ignored.

--

Created by Nelson Durrant, Oct 2024.
