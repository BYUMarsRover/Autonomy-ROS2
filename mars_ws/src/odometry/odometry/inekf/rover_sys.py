import numpy as np
from scipy.linalg import expm
from numpy.linalg import inv
from scipy.spatial.transform import Rotation as Rotation

class RoverSystem:
    def __init__(self):
        """System for a quadcopter, with IMU measurements as controls. 
        In the future this may be updated to represent the rover that has non-holonomic constraints,
        but that's not a trivial switch so for now it is a quadcopter.

        All noise values are set here, however this should be changed to be more robust
        """    

        # IMU noise values, these are the noise values for how noisy the imu is with regards to the motion update step
        std_gyro = 0.002
        std_a = 0.04
        std_a_bias = np.sqrt(0.01) #0.001
        std_gyro_bias = np.sqrt(0.02) #0.002

        self.Q = np.diag([std_gyro, std_gyro, std_gyro, std_a, std_a, std_a, 0, 0, 0, 
                    std_gyro_bias, std_gyro_bias, std_gyro_bias, std_a_bias, std_a_bias, std_a_bias])**2

        # Noise params in order of state (r, p, y, vx, vy, vz, x, y, z, gyro_bias, a_bias)
        # These are the noise values for how uncertain the rover is at startup
        self.init_noise = np.diag([np.deg2rad(10), np.deg2rad(10), np.deg2rad(50), 0, 0, 0, 0.5, 0.5, 0.5, 
                    std_gyro_bias, std_gyro_bias, std_gyro_bias, 1, 1, 1])**2


    def f_lie(self, state, u, dt, bias):
        """Propagates state forward in Lie Group. Used for IEKF.

        Args:
            state (5,5 ndarray) : X_n of model in Lie Group
            u     (2,3 ndarray) : U_n of model (IMU measurements)
            bias  (2,3 ndarray) : Estimated bias

        Returns:
            X_{n+1} (5,5 ndarray)"""

        g = np.array([0, 0, -9.8])
        R_90_ccw = Rotation.from_euler('z', 90, degrees=True).as_matrix()
        R = state[:3,:3]
        v = state[:3,3]
        p = state[:3,4]

        omega = u[0] - bias[0]
        a = u[1] - bias[1]
        
        ## put it together
        Rnew = R @ expm( self.cross( omega*dt ))
        vnew = v + (R@a + g)*dt
        pnew = p + v*dt + (R@a + g)*dt**2/2

        return np.block([[Rnew, vnew.reshape(-1,1), pnew.reshape(-1,1)],
                        [np.zeros((2,3)), np.eye(2)]])

    @staticmethod
    def cross(x):
        """Moves a 3 vector into so(3)

        Args:
            x (3 ndarray) : Parametrization of Lie Algebra

        Returns:
            x (3,3 ndarray) : Element of so(3)"""
        return np.array([[   0, -x[2],  x[1]],
                        [ x[2],     0, -x[0]],
                        [-x[1],  x[0],     0]])

    @staticmethod
    def carat(xi):
        """Moves a 9 vector to the Lie Algebra se_2(3).

        Args:
            xi (9 ndarray) : Parametrization of Lie algebra

        Returns:
            xi^ (5,5 ndarray) : Element in Lie Algebra se_2(3)"""
        w_cross = RoverSystem.cross(xi[0:3])
        v       = xi[3:6].reshape(-1,1)
        p       = xi[6:9].reshape(-1,1)
        return np.block([[w_cross, v, p],
                         [np.zeros((2,5))]])
    
    @staticmethod
    def adjoint(xi):
        """Takes adjoint of element in SE_2(3)

        Args:
            xi (5,5 ndarray) : Element in Lie Group

        Returns:
            Ad_xi (9,9 ndarray) : Adjoint in SE_2(3)"""
        R = xi[:3,:3]
        v_cross = RoverSystem.cross(xi[:3,3])
        p_cross = RoverSystem.cross(xi[:3,4])
        zero    = np.zeros((3,3))
        return np.block([[        R, zero, zero],
                         [v_cross@R,    R, zero],
                         [p_cross@R, zero,   R]])

