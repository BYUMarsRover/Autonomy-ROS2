# Rover custom heartbeat package

Custom package to provide a heartbeat for the rover and the base.

The base and rover each publish a message at `parameters.RATE` times per second.

Roverside, if a message hasn't been received for too long, the rover should stop motion, etc.
This is left up to each package to decide on tolerances and implementation. Note that as this package cannot guarantee node launch order, there may be "lost comms" for a little as all the nodes are launched. This package avoids that issue for the base by not publishing lost comms for the first `BASE_HEARTBEAT_WARMUP` seconds (set in `parameters.py`), but roverside, each package that uses the heartbeat must deal with this potential startup error.

On the base, if a message hasn't been received for too long, the operators should be alerted.
Tolerances for how long "too long" is are set in parameters.py.

You can use the heartbeat by including the following line in your rover launch file, the next
    in your base launch file, and subscribing to the appropriate status topic.

```
  <include file="$(find heartbeat)/launch/heartbeat_rover.launch"/>
```
```
  <include file="$(find heartbeat)/launch/heartbeat_base.launch"/>
```

Currently, these lines are included in `rover_common.launch` and `base_common.launch`, so the heartbeat statuses are available regardless of the task launched.

Here's how to run the node:
```
colcon build --packages-select heartbeat
source install/setup.bash
ros2 launch start heartbeat_base_launch.py
ros2 launch start heartbeat_rover_launch.py
```