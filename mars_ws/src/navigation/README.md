# Navigation
## Overview
The `navigation` package contains code used to run the rover's Autonomous Traversal in previous years. This includes a 2-part state machine (using 'states' and 'cartridges', where each state can run each cartridge). For the 2020 rover, we created a simplified, single state machine that can be found in the package `autonomous_state_machine`.

## Important Components
Some important files included in this package that are used in the autonomy task are:
|File|Description|
|---|---|
|`waypoint_manager.py`|Running the waypoint manager node|
|`ObstacleDetect.py`|Running the obstacle detection node|
|`base_autonomous.launch`|Launching the Autonomous Task GUI|

## Waypoint Manager
To run the waypoint manager, include the waypoint manager launch file in your package's launch file:
```
<include file="$(find navigation)/launch/waypoint_manager.launch" />
```

To run the waypoint manager from the command line, use `roslaunch`:
```
roslaunch navigation waypoint_manager.launch
```
### Important Note When Running Waypoint Manager
The waypoint manager requires messages to be published to the `/odometry/filtered` and `/ins/lla` topics in order to work. You can publish messages to these topics by [using Gazebo](#simulating-with-gazebo) or [running them with sensors plugged in](#using-physical-sensors).

#### Simulating with Gazebo
You can simulate these topics using Gazebo by running the customized Husky sim:
```
roslaunch husky_custom_gazebo husky_custom_empty_world.launch
```

#### Using Physical Sensors
You can get this to work with physical sensors by plugging in the ZED 2 camera and the UBLOX GPS module into USB ports on your development computer or on the rover. See the `README.md` in `/BYU-Mars-Rover/scripts/udev` if you have trouble getting these physical devices communicating with ROS.

### Testing Waypoint Manager Functionality
The easiest way to try out the waypoint manager is [using the Autonomous Task GUI](#using-the-autonomous-task-gui).

You should see non-zero values for the rover's current latitude and longitude (GPS will not work indoors, so values will be either 0 or whatever is in the cache if you are using the physical UBLOX sensor). Type some arbitrary distance and heading into the appropriate boxes under **Distance and Heading Conversion**, and click 'Conver to Lat/Lon'. You should now see the converted latitude and longitude in their respective boxes in the section **Latitude Longitude Coordinates**. Select 'Add Input Coordinates' to add them to the list of coordinates to be sent to the rover. Click 'SEND TO ROVER' to send them to the rover.

## Using Obstacle Avoidance
Launch the node in a launch file using arguments by including the following code in your launch files:
```
<arg name="display" default="false" />
<arg name="depth" default="true" />
<arg name="video_res" default="1080" />

<node pkg="navigation" type="ObstacleDetect.py" name="obstacle_detection" output="screen">
    <param name="display" value="$(arg display)" />
    <param name="depth" value="$(arg depth)" />
    <param name="video_height" value="$(arg video_res)" />
</node>
```

## Using the Autonomous Task GUI
Use `roslaunch` to launch the GUI:
```
roslaunch navigation base_autonomous.launch
```
**Note** There is an alias for this you can run instead:
```
base_autonomous
```

**Note** The first time you run the Autonomous Task GUI, you will see a blank screen. Select from the menu at the top Plugins > Rover / Misc > Coordinate Input. Also select Plugins > Rover / Misc > Rover Status.

Features:
* Click 'Convert to Lat/Lon' to convert a waypoint relative to the rover (defined with meters and a heading) to latitude/longitude
* Click 'Add Input Coordinates' to add the current lat/lon coordinates to the waypoint stack (doesn't send them to the rover yet)
* Double-click 'CLEAR ALL WAYPOINTS' to clear the waypoints from the Waypoints list (waypoint stack on the base station; not sent to the rover yet).
* Click 'Add Rover's Coordinates' to add the rover's current coordinates as a waypoint to the stack.
* Click 'Autonomous Mode' to toggle the rover's autonomous mode on or off.
* Click 'FULL AUTO RESET' to remove all the waypoints from the Navigation Queue (waypoint stack on the rover).