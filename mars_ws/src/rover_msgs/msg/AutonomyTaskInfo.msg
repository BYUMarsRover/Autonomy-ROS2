
# This message if for sending autonomy waypoint information

# This is used by AutonomyWaypoint.srv to send waypoints to the state machine.
# See AutonomyWaypoint.srv for more details

float64 latitude    # latitude
float64 longitude   # longitude
string tag_id       # tag id - 'GPS_only', '1', '2', '3', 'bottle', 'mallet'