import numpy as np
from numpy import sin, cos, sqrt

def convert_from_lidar_to_NED(self, point):
    """
    Converts a point from the LiDAR frame( X is up, Y is to the right of the rover, and Z is out away from the rover) to the NED (North East Down) frame.
    """
    #create rotation matrix from LiDAR frame to NED frame
    #Rotate back the current heading, and then rotate -90 degrees about y to the NED frame
    rot_matrix = self.rotx(self.rover_state_singleton.map_yaw) @ self.roty(-np.pi/2)

    #convert point to NED frame
    rot_point = rot_matrix @ point
    return rot_point

def convert_from_ZED_to_NED(self, point):
    """
    Converts a point from the ZED frame (X is forward, Y is to the left, and Z is up) to the NED (North East Down) frame.
    """
    #create rotation matrix from ZED frame to NED frame
    #Rotate back the current heading, and then rotate 1800 degrees about x to the NED frame
    rot_matrix = self.rotz(self.rover_state_singleton.map_yaw) @ self.rotx(np.pi)

    #convert point to NED frame
    rot_point = rot_matrix @ point
    return rot_point

## 3D Transformations
def rotx(self, th):
    """
    R = rotx(th)
    Parameters
        th: float or int, angle of rotation
    Returns
        R: 3 x 3 numpy array representing rotation about x-axis by amount theta
    """
    ## TODO - Fill this out
    R = np.array([[1, 0, 0],
                [0, cos(th), -sin(th)],
                [0, sin(th),  cos(th)]])

    return R    

def roty(self, th):
    """
    R = rotx(th)
    Parameters
        th: float or int, angle of rotation
    Returns
        R: 3 x 3 numpy array representing rotation about y-axis by amount theta
    """
    ## TODO - Fill this out
    R = np.array([[cos(th), 0, sin(th)],
                [0, 1, 0],
                [-sin(th), 0,  cos(th)]])

    return R

def rotz(self, th):
    """
    R = rotx(th)
    Parameters
        th: float or int, angle of rotation
    Returns
        R: 3 x 3 numpy array representing rotation about z-axis by amount theta
    """

    ## TODO - Fill this out
    R = np.array([[cos(th), -sin(th), 0],
                [sin(th), cos(th), 0],
                [0, 0,  1]])

    return R