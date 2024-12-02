# UBLOX_read_2

A library for parsing UBLOX packets and interfacing with UBLOX RTK GPS receivers in ROS2. It has been designed for use with the M8N and F9P GNSS receivers.

This library provides rather basic functionality and is designed to work under a linux environment, however it should probably work in Windows or Mac as it uses the cross-platform async_comm library as the serial interface.

The UBX parsing functionality is abstracted into a library for easy integration in other projects. Example usage is given in the main.cpp file.

Derived from https://github.com/byu-magicc/UBLOX_read which is the ROS1 version of this library.

Find additional software documentation here: https://wiki.magiccvs.byu.edu/#!sw_guides/ublox_read.md

Find additional hardware documentation here: https://wiki.magiccvs.byu.edu/#!hw_guides/ublox_f9p.md, https://wiki.magiccvs.byu.edu/#!hw_guides/c94_m8p.md

## Cloning the package

1. Install git if you haven't already done so. For example, on Ubuntu run the following:
```
sudo apt update
sudo apt upgrade -y
sudo apt install git -y
```

2. Clone the repository and the async_comm submodule:
```
git clone --recursive https://github.com/byu-magicc/UBLOX_read_2.git
```

Note that this guide does not use ROS2 workspaces, for simplicity. If you are familiary with ROS2 (and modifying dockerfiles, if using docker) then using a workspace structure is recommended, but not required.

## Running with a native ROS2 installation

To run this on a computer with ROS2 installed, use these directions.

To install ROS2, refer to the [ROS2 documentation](https://docs.ros.org/). If you are running Ubuntu 22.04 and want to install ROS2 Humble, follow [this guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html). Make sure to install `ros-dev-tools` in addition to `ros-humble-desktop` or `ros-humble-ros-base`.

1. Navigate to the repository.
```
cd <path to repo>
```

2. Install all the necessary dependencies.
```
rosdep install --from-paths . --ignore-src -y
```

3. Build the workspace.
```
colcon build
```

4. Source your workspace setup file. Add this source command to your .bashrc or .zshrc if you don't want to source the setup file in every terminal.
```
source <path to repo>/install/setup.bash
```

## Running within a docker container on a Linux system

If you don't want to install ROS2 or aren't running Ubuntu, you can run this library in a docker container. The included Dockerfile and compose.yaml file should work out of the box for Linux. Windows or MacOS users will probably need to edit these files to get them to work.

You will need to install docker engine if you haven't already. For Ubuntu, see this [guide](https://docs.docker.com/engine/install/ubuntu/).

1. Navigate to the root of the UBLOX_read_2 repository.
```
cd <path to repo>
```

2. Build the docker image. This only needs to be done once, so if you are re-running UBLOX_read_2 after setting it up, you can skip this step.
```
docker compose build
```

3. Once built, start the docker container as a background daemon.
```
docker compose up -d
```

4. Open a bash terminal in the docker container.
```
docker compose exec ublox_read_2 bash
```

5. Build the workspace.
```
colcon build
```

6. Source the workspace setup file. This should already be added to the .bashrc file, so alternatively you could leave and re-enter the docker container with ctrl+d.
```
source /ublox_read_2/install/setup.bash
```

## Basic Setup

To perform a basic setup with one base station RTK GPS (base) and one moving RTK GPS (rover), follow these instructions.

1. Edit the rover_launch.xml file on the rover and base_launch.xml file on the base, found in the launch folder. Update the base_host ip address and the rover_host ip address to reflect the actual ip addresses of both the rover and base computers. Also update the serial_port with the actual address for your GPS module on both the base and rover.

2. You may need to rebuild your workspace in order for the modified launch files to take effect. (Using a symbolic colcon install may also fix this issue.)
```
colcon build
```

3. Run the launch files with these commands on the base station and rover.
```
ros2 launch ublox_read_2 base_launch.xml
```
```
ros2 launch ublox_read_2 rover_launch.xml
```

4. If everything is working then you should see GPS-related topics show up with `ros2 topic list` and the `ros2 topic echo /rover/RelPos` should be publishing relative postion values, once the base station has surveyed in (check `/base/Survey` for survey status).

### Tips

To find all the ip addresses on your local network, first use `ip addr` to find your current ip address. Then use `nmap -sn 192.168.0.104/24` to scan all ip addresses on your network, replacing `192.168.1.118` with your ip address. You may need to install `nmap`.

To find the serial port of your GPS module, run `ls /dev/tty*` with your module unplugged. Then plug it in and re-run the command. The serial address that showed up when your GPS was plugged in should be your GPS. If nothing changed, check to make sure Linux sees your USB device with `lsusb`. If `lsusb` returns nothing, then you have a problem with your Linux configuration or your GPS module (or USB cable).

To record GPS data, you can do this with a ROS bag. To record all published messages in a ROS bag, use this command.
```
ros2 bag record -a
```
