'''
Device Name Map

Map symlinks to printable string on the GUI for all the devices that we 
currently use. It is assumed that the '/dev/rover' part of the pathname
is omitted.
'''

ROVER_DEV_NAME_MAP = {
    "onBoardMega" : "Arduino Mega (Wheels)",
    "peripheralsBoard" : "Arduino Nano (Elev.,Grip.,Bat.,LEDs)",
    "rtk" : "RTK GPS (rover)",
    "cameras/ZED_front" : "ZED",
    "cameras/lowerFrontMastCam" : "Low Front Camera",
    "cameras/birdCam" : "Bird's Eye View Camera",
    "cameras/rearBirdCam" : "Rear Bird's Eye View Camera",
    "cameras/clickerCam" : "Clicker Camera",
    "cameras/hexKeyCam" : "Hex Key Camera",
    "cameras/gripperCam" : "Gripper Camera",
    "scienceArduinoNano" : "Science Arduino",
    "cameras/microscopeCam" : "Science Microscope",
    "cameras/fadCam" : "FAD Camera",
    "cameras/autonomyWebCam" : "Autonomy Webcam",
    "cameras/scienceLegCam" : "Science Leg Camera",
}

BASE_DEV_NAME_MAP = {
    "js/xbox_one_drive" : "Xbox Controller (drive)",
    "js/xbox_one_arm" : "Xbox Controller (arm)",
    "onBoardMega" : "Arduino Mega (Antenna)",
    "rtk" : "RTK GPS (base station)",
}
