## Autonomy-ROS2 Guide

A simple guide to working with the Autonomy-ROS2 repository.

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

**Helpful Resources:**

GitHub Quickstart Tutorial - [https://docs.github.com/en/get-started/start-your-journey/hello-world](https://docs.github.com/en/get-started/start-your-journey/hello-world)

Tmux Cheat Sheet - [https://tmuxcheatsheet.com/](https://tmuxcheatsheet.com/)

Docker Concepts and Tutorials - [https://docs.docker.com/get-started/introduction/whats-next/](https://docs.docker.com/get-started/introduction/whats-next/)

ROS 2 Concepts - [https://docs.ros.org/en/humble/Concepts/Basic.html](https://docs.ros.org/en/humble/Concepts/Basic.html)

ROS 2 CLI Tool Tutorials - [https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools.html](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools.html)

ROS 2 Code Development Tutorials - [https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries.html](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries.html)

Robotics in ROS 2 Tutorials - [https://github.com/henki-robotics/robotics_essentials_ros2/tree/main](https://github.com/henki-robotics/robotics_essentials_ros2/tree/main)

--

Created by Nelson Durrant, Feb 2025.
