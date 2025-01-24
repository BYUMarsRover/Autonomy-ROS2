## Domain Bridge Guide

This guide outlines the steps to set up and run a domain bridge between different ROS domains.

--

**Preparation:**

- Add topic mapping declarations to `mars_bridge_config.yaml`. Refer to `example_bridge_config.yaml` for examples of different configurations.

--
  
**Running the Bridge:**

- Run `bash compose.sh` to launch and enter the Docker container.

- Run `bash run_bridge.sh` from inside the container to run the bridge.

--

Created by Nelson Durrant, Jan 2025.