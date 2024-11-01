int8 id
int8 label
float64 confidence
# Position is given in millimeters from the left camera, with a east-down-north coordinate system,
# facing the same direction as the camera (the default COORDINATE_SYSTEM_IMAGE for the ZED SDK:
# https://www.stereolabs.com/docs/positional-tracking/coordinate-frames/).
float64 x
float64 y
float64 z