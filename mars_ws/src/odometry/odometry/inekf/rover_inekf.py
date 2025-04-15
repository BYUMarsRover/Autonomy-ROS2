import numpy as np
from scipy.spatial.transform import Rotation as Rotation
from scipy.linalg import expm, logm
from numpy.linalg import inv
import math

"""InEKF class for the BYU Mars rover. This class implements the InEKF algorithm for state estimation.
Author: Derek Benham
Date: April 2025

It uses IMU measurements for prediction and GPS measurements for correction.
The state is represented in a Lie group, and the algorithm uses the exponential map for state propagation.
The class also handles bias estimation for the IMU measurements.

For more information on the InEKF algorithm, please see the following papers:
An Introduction to the Invariant Extended Kalman Filter [Lecture Notes]: Easton R. Potokar, Randal W. Beard, Joshua G. Mangelson
Invariant Extended Kalman Filter for Autonomous Surface Vessels with Partial Orientation Measurements: Derek Benham, Easton Potokar, Joshua G. Mangelson

If this code is used for research purposes, please cite the following paper:
Invariant Extended Kalman Filter for Autonomous Surface Vessels with Partial Orientation Measurements
<TODO: Insert full citation here once published>
"""

def pose_matrix_inv(pose):
    """
    Inverts a 5x5 pose matrix in SE_2(3).
    inputs:
        pose: 5x5 pose matrix in SE_2(3)
    outputs:
        pose_inv: 5x5 inverted pose matrix in SE_2(3)
    """
    pose_inv = np.zeros((5,5))
    pose_inv[:3,:3] = pose[:3,:3].T
    pose_inv[:3,3] = -pose[:3,:3].T @ pose[:3,3]
    pose_inv[:3,4] = -pose[:3,:3].T @ pose[:3,4]
    pose_inv[3:5, 3:5] = np.eye(2)
    return pose_inv

class InEKF:
    def __init__(self, sys, est_bias=True):
        '''
        InEKF class
        This class implements the InEKF algorithm for state estimation.
        Inputs:
            sys (rover_sys.py) : System object containing the system model and noise parameters
            est_bias (bool) : Whether to account for IMU bias in the filter. If using real world data, set to True
        '''
        self.est_bias = est_bias
        self.sys = sys # import system object (rover_sys.py)

        self.prev_time = 0

        self.filter_started = False

        self.mus = []
        self.biases = []
        self.sigmas = [self.sys.init_noise]
        self.timestamps = []

    def initialize_filter(self, init_state, first_timestamp=0):
        """
        Filter init state function call. Broken out from __init__ for implementation ease as init location is unknown at system startup

        :init_state: 9 element array of initial state [roll,pitch,yaw,vx,vy,vz,px,py,pz]
        :first_timestamp: timestamp of first measurement in seconds (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        
        :return: None
        """ 
        mu0 = init_state

        if mu0.shape ==(9,):
            mu0 = np.block([[Rotation.from_euler('xyz', mu0[:3]).as_matrix(), mu0[3:6].reshape(-1,1), mu0[6:9].reshape(-1,1)],
                            [np.zeros((2,3)), np.eye(2)]])  
        
        self.mus = [mu0]
        self.timestamps = [first_timestamp]
        self.prev_time = first_timestamp
        self.biases = [np.zeros((2,3))]
        self.filter_started = True

    # Helper functions 
    @property
    def mu(self):
        return self.mus[-1]
    
    @property
    def sigma(self):
        return self.sigmas[-1]
    
    @property
    def bias(self):
        return self.biases[-1]

    # InEKF Propagate motion
    # Process IMU data for prediction step
    def imu_callback(self, time, angular_velocity, linear_accel):
        """Runs prediction step of InEKF

        :time: timestamp of IMU measurement in seconds (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        :angular_velocity: angular velocity in rad/s [omega_x, omega_y, omega_z]
        :linear_accel: linear acceleration in m/s^2 [accel_x, accel_y, accel_z]
        
        :return:None
        """
                
        current_time = time
        
        dt = current_time - self.prev_time
        self.prev_time = current_time

        angular_velocity = np.array(angular_velocity)
        linear_accel = np.array(linear_accel)

        u = np.array([angular_velocity, linear_accel])
    
        # Update the state
        mu_bar = self.sys.f_lie(self.mu, u, dt, self.bias)

        # make propagation matrix
        zero = np.zeros((3,3))
        I = np.eye(3)

        # Get A matrix
        w_cross = self.sys.cross(angular_velocity - self.bias[0])
        a_cross = self.sys.cross(linear_accel - self.bias[1])

        # Left error A matrix
        self.expA = expm(np.block([[  -w_cross,      zero,      zero,    -I,  zero],
                                [  -a_cross,  -w_cross,      zero,  zero,    -I],
                                [      zero,         I,  -w_cross,  zero,  zero],
                                [      zero,      zero,      zero,  zero,  zero],
                                [      zero,      zero,      zero,  zero,  zero]])*dt)
        
        sigma_bar = self.expA @ self.sigma @ self.expA.T + self.expA @ self.sys.Q @ self.expA * dt
        

        # save for use later
        self.mus.append( mu_bar )
        self.biases.append( self.bias )
        self.sigmas.append( sigma_bar )
        self.timestamps.append(current_time)

    def heading_callback(self, heading, heading_acc): # DGPS callback
        """Runs Update step for a heading only measurement (rotation about Z-axis)

        :heading: rotation about world Z-axis in degrees
        :heading_acc: std_dev of the heading measurement in degrees
        
        :return:None
        """
        # rpy = Rotation.from_euler('z', heading, degrees=True).as_matrix()

        R_hat = self.mu[:3,:3]        
        
        # yaw_dgps_meas = Rotation.from_matrix(rpy).as_euler('xyz')[2]
        yaw_dgps_meas = np.deg2rad(heading)
        so3_rotation_yaw_dgps_meas = Rotation.from_euler('z', yaw_dgps_meas).as_matrix()

        veh_yaw = Rotation.from_matrix(R_hat).as_euler('xyz')[2]
        veh_yaw_only = Rotation.from_euler('xyz', [0, 0, veh_yaw]).as_matrix()
        veh_no_yaw = veh_yaw_only.T @ R_hat
        z = so3_rotation_yaw_dgps_meas @ veh_no_yaw

        z_hat = veh_yaw_only
        z = so3_rotation_yaw_dgps_meas

        zero = np.zeros((3,3))
        I = np.eye(3)
        H = np.block([[-I, zero, zero, zero, zero]]) # Measurement matrix, measurement is rpy
        
        V_log = logm(z.T@z_hat) # innovation is measured pitch^-1 @ belief pitch
        V = np.array([V_log[2, 1], V_log[0, 2], V_log[1,0]]) # Extract the log of the rotation matrix using the Vee operator

        # make our special measurement covariance
        invR = np.zeros((3,3))
        invR[2][2] = 1/np.deg2rad(heading_acc)**2

        sig_til = inv(H@self.sigma@H.T)
        S_inv = sig_til - sig_til@inv(R_hat.T@invR@R_hat + sig_til)@sig_til

        K = self.sigma @ H.T @ S_inv # Kalman gain

        K_V = K @ V
        
        # Update the state
        self.mus[-1][:3,:3] = self.mu[:3,:3] @ expm( self.sys.cross( K_V[:3] ) )
        self.mus[-1][:3,3] += K_V[3:6] # update velocity
        self.mus[-1][:3,4] += K_V[6:9] # update position
        
        if self.est_bias:
            self.biases[-1] = self.bias + ( K_V[9:] ).reshape((2,3)) # update bias
        
        self.sigmas[-1] = (np.eye(15) - K @ H) @ self.sigma # Update the covariance
        return 0

    def gps_callback(self, x, y, alt, hor_acc, vert_acc):
        """Runs Update step for a GPS XYZ measurement (in meters)

        :x: x position in meters
        :y: y position in meters
        :alt: z position in meters (be mindful of frame consistency such as waterline height or ellipsoid height)
        :hor_acc: horizontal std dev in meters
        :vert_acc: vertical std dev in meters

        :return:None
        """

        R_hat = self.mu[:3,:3]

        zero = np.zeros((3,3))
        I = np.eye(3)
        H = np.block([[zero, zero, I, zero, zero]]) # Measurement matrix, measurement is xyz for now

        R_pos_sensor = np.diag([hor_acc, hor_acc, vert_acc])**2
        S = H @ self.sigma @ H.T + R_hat.T @ R_pos_sensor @ R_hat

        K = self.sigma @ H.T @ inv(S)

        zero = np.zeros((3,2))
        II = np.block([[I, zero]])
        Z = np.array([[x, y, alt, 0, 1]]).T

        V = II @ pose_matrix_inv(self.mu) @ Z

        self.mus[-1] = self.mu @ expm( self.sys.carat( (K[:9] @ V).squeeze() ))
        
        if self.est_bias:
            self.biases[-1] = self.bias + ( (K[9:] @ V).squeeze() ).reshape((2,3)) # update bias
        
        self.sigmas[-1] = (np.eye(15) - K @ H) @ self.sigma # Update the covariance

    def gps_velocity_callback(self, vel_enu, vel_acc):
        """Runs Update step for a GPS Velocity measurement

        :vel_enu: velocity in the East North Up frame [vx, vy, vz]
        :vel_acc: std_dev of the velocity measurement in meters/second. Float

        :return:None
        """

        Rot = self.mu[:3,:3]

        zero = np.zeros((3,3))
        I = np.eye(3)
        H = np.block([[zero, I, zero, zero, zero]]) # Measurement matrix

        R_pos_sensor = np.eye(3)*vel_acc**2
        R_pos_sensor[2][2] /= 4 # Set the vertical measurement variance to a quarter of the actual
        # Hacky, but I found it gives good results on the rover

        S = H @ self.sigma @ H.T + Rot.T @ R_pos_sensor @ Rot

        K = self.sigma @ H.T @ inv(S)

        zero = np.zeros((3,2))
        II = np.block([[I, zero]])
        Z = np.array([[vel_enu[0], vel_enu[1], vel_enu[2], 1, 0]]).T

        V = II @ pose_matrix_inv(self.mu) @ Z

        self.mus[-1] = self.mu @ expm( self.sys.carat( (K[:9] @ V).squeeze() ))
        
        if self.est_bias:
            self.biases[-1] = self.bias + ( (K[9:] @ V).squeeze() ).reshape((2,3)) # update bias
        
        self.sigmas[-1] = (np.eye(15) - K @ H) @ self.sigma # Update the covariance
        return 

    ############################################################################################################
    # The below functions are not currently in use, but are left here for future reference
    # I couldn't get them integrated properly and it's possible it was simply a frame issue
    
    # Both functions will need to be updated to handle variable noise params, currently hardcoded in this file
    ############################################################################################################

    def full_orientation_callback(self, time, R_mat):
        """Runs Update step for full orientation measurement

        :time: timestamp of IMU measurement in seconds (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        :R_mat: 3x3 ndarray of rotation matrix as estimated by the IMU
        
        :return:None
        """

        # 90 degree CCW rotation matrix about Z-axis
        R_z_90 = np.array([
            [0, -1, 0],
            [1,  0, 0],
            [0,  0, 1]
        ])
        R_mat = R_z_90 @ R_mat # Rotate 90 degrees CCW about Z-axis to match frames

        R_hat = self.mu[:3,:3]        
    
        z = R_mat
        z_hat = R_hat
        
        zero = np.zeros((3,3))
        I = np.eye(3)
        H = np.block([[-I, zero, zero, zero, zero]]) # Measurement matrix, measurement is rpy
        
        V_log = logm(np.linalg.inv(z)@z_hat) # innovation is measured pitch - belief pitch
        V = np.array([V_log[2, 1], V_log[0, 2], V_log[1,0]])#.reshape(3,1) # Extract the log of the rotation matrix using the Vee operator

        # make our special measurement covariance
        sigma_S = np.linalg.inv(H @ self.sigma @ H.T)
        R_orientation_sensor_noise = np.diag([np.deg2rad(5), np.deg2rad(5), np.deg2rad(5)])
        S = sigma_S - sigma_S@np.linalg.inv(R_hat.T@R_orientation_sensor_noise@R_hat)@sigma_S

        K = self.sigma @ H.T @ inv(S) # Kalman gain

        K_V = K @ V
        
        # Update the state
        self.mus[-1][:3,:3] = self.mu[:3,:3] @ expm( self.sys.cross( K_V[:3] ) )
        self.mus[-1][:3,3] += K_V[3:6] # update velocity
        self.mus[-1][:3,4] += K_V[6:9] # update position
        
        if self.est_bias:
            self.biases[-1] = self.bias + ( K_V[9:] ).reshape((2,3)) # update bias
        
        self.sigmas[-1] = (np.eye(15) - K @ H) @ self.sigma # Update the covariance


    def partial_orientation_gravity_callback(self, linear_acceleration):
        """Runs Update step for partial orientation measurement of roll and pitch
        The intuition is that in the IMU frame, the Z-axis is aligned with gravity, so we can use the linear acceleration to estimate roll and pitch.
        This assumption is only valid when other forces are minimal (e.g. when the vehicle is not accelerating or turning).

        :time: timestamp of IMU measurement in seconds (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        :linear_acceleration: 3x1 ndarray of linear acceleration in the IMU frame [ax, ay, az]
        
        :return:None
        """
        # Convert linear acceleration to a roll pitch measurement
        roll = math.atan2(linear_acceleration[1], linear_acceleration[2])
        pitch = math.atan2(linear_acceleration[0], linear_acceleration[2])  # You might need to adjust the sign of ax

        r, p, y = Rotation.from_matrix(self.mu[:3,:3]).as_euler('xyz')
        
        R_p_orientation_measurement = self.rotation_matrix_from_roll_pitch(roll, pitch)
        # R_cam_measurement = Rotation.from_euler('xyz', [roll, pitch, y]).as_matrix()

        R_hat_yaw_only = Rotation.from_euler('xyz', [0, 0, y]).as_matrix()
        R_p_orientation_estimate = R_hat_yaw_only.T @ self.mu[:3,:3] # Converting estimate to seafaring frame


        z = R_p_orientation_measurement
        
        R_bel = R_p_orientation_estimate
        
        zero = np.zeros((3,3))
        I = np.eye(3)
        H = np.block([[-I, zero, zero, zero, zero]]) # Measurement matrix, measurement is rpy
        # H[2][2] = 1e-8

        z_hat = R_bel # Predicted measurement
        
        V_log = logm(np.linalg.inv(z)@z_hat) # innovation is measured pitch - belief pitch
        V = np.array([V_log[2, 1], V_log[0, 2], V_log[1,0]])#.reshape(3,1) # Extract the log of the rotation matrix using the Vee operator

        invR = np.zeros((3,3))
        invR[0][0] = 1/np.deg2rad(15)**2
        invR[1][1] = 1/np.deg2rad(15)**2

        sig_til = inv(H@self.sigma@H.T)

        sig_til_no_inv = H@self.sigma@H.T
        S_inv = sig_til - sig_til@inv(sig_til_no_inv @ (R_bel.T@invR@R_bel)+np.eye(3))

        K = self.sigma @ H.T @ S_inv #inv(S) # Kalman gain

        K_V = K @ V
        
        # Update the state
        self.mus[-1][:3,:3] = self.mu[:3,:3] @ expm( self.sys.cross( K_V[:3] ) )
        self.mus[-1][:3,3] += K_V[3:6] # update velocity
        self.mus[-1][:3,4] += K_V[6:9] # update position
        
        if self.est_bias:
            self.biases[-1] = self.bias + ( K_V[9:] ).reshape((2,3)) # update bias
        
        self.sigmas[-1] = (np.eye(15) - K @ H) @ self.sigma # Update the covariance


def main(args=None):
    pass