# Path Planning - USE

Description:
The path planning library has one ROS2 node that is used for two primary path planning functions.
1. The first orders up to 8 waypoints in a way that corresponds to the shortest path to visit all of them.
2. The second function is to plan a path between where the rover currently is, and a single waypoint.

Typical Use:
1. The waypoints are added to the autonomy gui via user entry (see documentation on the autonomy gui).
2. Click reorder to command the path planner to reorder the waypoints. This reordering will reflect on the gui waypoints.
3. Select the radio button of the waypoints you want to plan a path to and click plan path.
4. Once the path status displays READY, you can click "Send Waypoint" and the state machine will receive the path.
5. Then enable Autonomy and go get points!

Critical Details:
1. The the rover state singleton topic (rover location and orientation) must publish for the path planning node to pick which topological map to use (based on where the rover says it is at).
2. For either of the above path planning functions to work, you must have a topological map that captures all GPS coordinates that will be used in the task.
There is a hanksville full LiDAR map that can be chopped to the location you need.
