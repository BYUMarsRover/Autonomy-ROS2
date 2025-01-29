## Simple Domain Bridge Guide

This guide outlines the steps to set up and run a domain bridge between the rover and base station.

> **IMPORTANT:** I set up this bridge in an attempt to limit communication between nodes on the rover and base station due to antenna bandwidth constraints. IT DID NOT WORK FOR THAT. We actually noted a large increase in bandwidth usage when running the bridge. I'm leaving the code here for reference in the future, but it is not recommended for use in the current project.
> 
> - Nelson Durrant, Jan 2025

--

**Preparation:**

- Set desired ROS_DOMAIN_ID values for the rover and base station in the `mars_bridge_config.yaml` file.

- Add topic mapping declarations to `mars_bridge_config.yaml`. Refer to `example_bridge_config.yaml` for examples of different configurations.

--
  
**Running the Bridge:**

- Launch ROS 2 nodes on the rover and base station with the desired ROS_DOMAIN_ID values, using `export ROS_DOMAIN_ID=<selected_id>`.

- Run `bash compose.sh` to launch and enter the Docker container (unless you're already inside).

- Run `bash run_bridge.sh` from inside the container to run the bridge.

--

Created by Nelson Durrant, Jan 2025.