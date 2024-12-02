"""
Contains constants for the Heartbeat source code.
"""

# How often the base will ping the rover and vice versa.
RATE = 10  # Hz
# How long since last msg from rover before displaying warning to operators
#   and publishing lost comms string.
BASE_WARNING_TOLERANCE = 1.0  # seconds
BASE_HEARTBEAT_WARMUP = 10.0  # seconds
BASE_WARNING_STRING = (
    "LOST COMMS: >" + str(BASE_WARNING_TOLERANCE) + "s since last response from rover."
)