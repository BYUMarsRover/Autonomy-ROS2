import numpy as np

def quat2R(q):
    """
    R = quat2R(q)
    Description:
    Returns a 3x3 rotation matrix

    Parameters:
    q - 4x1 numpy array, [nu, ex, ey, ez ] - defining the quaternion
    
    Returns:
    R - a 3x3 numpy array 
    """
    # TODO, extract the entries of q below, and then calculate R
    [ex, ey, ez, nu] = q
    R = np.array([[2*(nu**2 + ex**2) - 1, 2*(ex*ey - nu*ez), 2*(ex*ez + nu*ey)],
                  [2*(ex*ey + nu*ez), 2*(nu**2 + ey**2) - 1, 2*(ey*ez - nu*ex)],
                  [2*(ex*ez - nu*ey), 2*(ey*ez + nu*ex), 2*(nu**2 + ez**2) - 1]])
    return R

def R2quat(R):
    """
    quaternion = R2quat(R)
    Description:
    Returns a quaternion representation of pose

    Parameters:
    R

    Returns:
    quaternion - 1 x 4 numpy matrix, quaternion representation of pose in the 
    format [nu, ex, ey, ez]
    """
    # TODO, see equation (2.34) and (2.35) on pg. 55, using functions like "sp.sqrt," and "sp.sign"

    [[r11, r12, r13],
     [r21, r22, r23],
     [r31, r32, r33]] = R

    return np.array([0.5*np.sign(r32 - r23)*np.sqrt(r11 - r22 - r33 + 1),
                     0.5*np.sign(r13 - r31)*np.sqrt(r22 - r33 - r11 + 1),
                     0.5*np.sign(r21 - r12)*np.sqrt(r33 - r11 - r22 + 1),
                     0.5*np.sqrt(r11 + r22 + r33 + 1)])