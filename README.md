## Autonomy-ROS2 Guide

This README gives a simple guide to and overview of the Autonomy-ROS2 repository.

--

**Getting Started:**

- Install WSL2 on your Windows machine by following the instructions [here](https://docs.microsoft.com/en-us/windows/wsl/install).

- Install Docker Desktop on your Windows machine by following the instructions [here](https://docs.docker.com/desktop/), and enable the WSL 2 backend by following the instructions [here](https://docs.docker.com/desktop/windows/wsl/).

- Open a WSL terminal and clone this repo into your Linux environment using `git clone https://github.com/BYUMarsRover/Autonomy-ROS2.git`.

- Run `bash compose.sh` to launch and enter the Docker container. This should pull the latest Docker image from Docker Hub, and might take a couple minutes the first time you run it.

> **NOTE:** 
>
> `compose.sh` spins up a Docker container from our custom ROS 2 image, which includes all the necessary dependencies needed to run our ROS 2 packages. If you make changes inside the Docker container (i.e. installing packages or modifying files not included as volumes), those changes will not be saved when the container restarts. If you ever need to restart the container, simply run `bash compose.sh down` to stop the container, and then `bash compose.sh` to start it again.

--

**Contributing:**

- **Create a new branch.** The main branch of this repository is protected, so you will need to create a new branch to make changes. To do this, run `git checkout -b <branch_name>`. We recommend naming your branch with a combination of your name and the feature you are working on (i.e. `nelson/repo-docs`).

- **Make your changes.** Add new ROS 2 packages to the `mars_ws/src` folder, or modify existing packages as needed. If you need to add dependencies to the Docker image, modify `docker/Dockerfile` (these changes will not be pushed to Docker Hub until you merge your branch into the main branch).

- **Submit a pull request.** Once you have made and tested your changes, make sure they are commited and pushed. Then, navigate to the GitHub repository and create a new pull request. Once another member of the team has reviewed and approved your changes, you can merge them into the main branch.

--

**Structure:**

- `Autonomy-ROS2/docker`: Contains the Dockerfile and docker compose file for building and running our custom ROS 2 Docker image. If you need to add dependencies to the image, simply add them to `docker/Dockerfile`, push them to the main branch, and the GitHub Actions workflow will automatically rebuild and redeploy the image to Docker Hub.

- `Autonomy-ROS2/mars_ws`: The ROS 2 workspace we use for building and running ROS 2 packages. You can add custom ROS 2 packages to the `src` folder in this workspace and build them using the `colcon` build system.

- `Autonomy-ROS2/ros1_bridge`: Contains the code for running the ROS 1 - ROS 2 bridge. It has a seperate, more complicated Docker image and ROS workspace setup. For more information on how to work with the bridge, refer to the `ros1_bridge/README.md` file.

--

Created by Nelson Durrant, Feb 2025.