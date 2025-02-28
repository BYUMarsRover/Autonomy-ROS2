#!/usr/bin/python3

import math
import rclpy
from rclpy.node import Node

# Monkey patch (there is an bug in the transforms3d library within tf_transformations that uses np.float instead of float)
import numpy as np
np.float = float  # Temporary alias for compatibility

# from tf_transformations import euler_from_quaternion, quaternion_multiply

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from odometry.transform_tools import quat2R, R2quat

from geometry_msgs.msg import TransformStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from rover_msgs.msg import RoverStateSingleton
from sensor_msgs.msg import NavSatFix, Imu
import numpy as np

from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy

np.float = float # Bug fix hack

def euler_from_quaternion(quat):
    """Convert a quaternion into euler angles (roll, pitch, yaw).
    
    quat: [x, y, z, w]
    returns: (roll, pitch, yaw) in radians
    """

    x, y, z, w = quat
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

def quat_to_list(quat):
    return [quat.x, quat.y, quat.z, quat.w]

def list_to_quat(list):
    quat = Imu().orientation
    quat.x = list[0]
    quat.y = list[1]
    quat.z = list[2]
    quat.w = list[3]
    return quat

def quaternion_multiply(q1, q2):
    """Multiply two quaternions."""
    q1 = quat_to_list(q1)
    q2 = quat_to_list(q2)
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return list_to_quat(np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ]))

class RoverStateSingletonCreator(Node):
    """
    This class subscribes to gps data and zed orientation data for lat/lon and zed_yaw
    """
    def __init__(self):
        super().__init__('rover_state_singleton_creator')

        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,  # Keeps only the latest message
            depth=1,  # Depth of 1 ensures only the latest message is kept
            reliability=QoSReliabilityPolicy.BEST_EFFORT  # Best effort reliability
        )

        # TF listner
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribers
        # USED THIS SUBSCRIBER when trusting the zed orientation
        # self.zed_orientation_subscription = self.create_subscription(Odometry, "/zed/odom", self.convert_map, 10)  # Subscribes to the ZED orientation data
        # USED THIS SUBSCRIBER WHEN trusting the filter madgewick
        self.imu_filter_sub = self.create_subscription(Vector3Stamped, "/imu/rpy/filtered", self.set_map_yaw, qos_profile)

        # TODO: Chekc the filtered GPS subscription
        self.unfiltered_gps_subscription = self.create_subscription(NavSatFix, "/fix", self.convert_unfiltered_gps, qos_profile)  # Subscribes to the filtered GPS data from the UKF output
        # self.gps_subscription = self.create_subscription(NavSatFix, "/ins/lla", self.convert_gps, qos_profile) # Subscribes to unfiltered GPS data
        self.gps_subscription = self.create_subscription(NavSatFix, "/zed/global", self.convert_gps, qos_profile) # Subscribes to zed filtered GPS data

        # Publishers
        self.singleton_publisher = self.create_publisher(RoverStateSingleton, '/odometry/rover_state_singleton', qos_profile) # Publishes the singleton message

        # timer
        self.transform_lookup = self.create_timer(10.0, self.transform_check)

        self.map_roll = 0.0
        self.map_pitch = 0.0
        self.map_yaw = 0.0

        self.odom_roll = 0.0
        self.odom_pitch = 0.0
        self.odom_yaw = 0.0

        self.map_x = 0.0
        self.map_y = 0.0
        self.map_z = 0.0

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_z = 0.0

        self.map_x_dot = 0.0
        self.map_y_dot = 0.0
        self.map_z_dot = 0.0

        self.odom_x_dot = 0.0
        self.odom_y_dot = 0.0
        self.odom_z_dot = 0.0

        self.map_roll_dot = 0.0
        self.map_pitch_dot = 0.0
        self.map_yaw_dot = 0.0

        self.odom_roll_dot = 0.0
        self.odom_pitch_dot = 0.0
        self.odom_yaw_dot = 0.0

        self.gps = NavSatFix()
        self.raw_gps = NavSatFix()

        self.t = TransformStamped()

    def transform_check(self):
        try:
            self.t = self.tf_buffer.lookup_transform(
            'odom',
            'map',
            rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform map to odom: {ex}')
            return
    

    def set_map_yaw(self, msg):
        #Take the euler angle from the madgewick filter directly and set the yaw

        self.map_yaw = msg.vector.z * (180.0 / math.pi)

        self.publish_message()
    
    
    def convert_map(self, message):

        """
        Converts orientation from quaternions to euler (converts them into human readable roll, pitch, yaw) angles and sets the map variables
        """
        
        orientation_q = quaternion_multiply(self.t.transform.rotation, message.pose.pose.orientation)  # Extracts the orientation data from the message
        # orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]  # Extracts quaternion data

        # R = quat2R(orientation_list)

        # R_rot = np.array([[ 0,  0, 1],
        #                   [-1,  0, 0],
        #                   [ 0, -1, 0]])

        # R = R_rot @ R

        # orientation_list = R2quat(R)
        w = orientation_q.w
        x = orientation_q.x
        y = orientation_q.y
        z = orientation_q.z
        # yaw = np.atan2(2.0 * (y*z + w*x), w*w - x*x - y*y + z*z)
        yaw = math.atan2(2.0 * (w*z + x*y), 1.0 - 2.0 * (y* y + z * z))

        # dcm10 = 2.0 * (x * y + w * z)
        # dcm00 = w * w + x * x - y * y - z * z

        # yaw = math.atan2(dcm10, dcm00)


        # euler = euler_from_quaternion(orientation_list, axes ='rzyx')  # Transforms the quaternion data into euler angles

        # sets the roll, pitch and yaw based on the euler angles and converts them to degrees
        # zed_roll = euler[0] * 180/math.pi
        # zed_pitch = euler[1] * 180/math.pi
        # zed_yaw = euler[2] * 180/math.pi


        # self.map_roll = euler[0] * 180/math.pi
        # self.map_pitch = euler[1] * 180/math.pi
        # self.map_yaw = euler[2] * 180/math.pi 
        self.map_yaw = (yaw * 180/math.pi )
        self.get_logger().info(f"MAP YAW: {self.map_yaw}")

        if self.map_yaw < -180:
            self.map_yaw += 360

        self.publish_message()


    def convert_odom(self, message):
        """
        Converts orientation from quaternions to euler (converts them into human readable roll, pitch, yaw) angles and sets the odom variables
        """
        orientation_q = message.pose.pose.orientation  # Extracts the orientation data from the message
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]  # Extracts quaternion data
        euler = euler_from_quaternion(orientation_list)  # Transforms the quaternion data into euler angles

        # sets the roll, pitch and yaw based on the euler angles and converts them to degrees
        self.odom_roll = euler[0] * 180/math.pi
        self.odom_pitch = euler[1] * 180/math.pi
        self.odom_yaw = euler[2] * 180/math.pi

        self.odom_x = message.pose.pose.position.x
        self.odom_y = message.pose.pose.position.y
        self.odom_z = message.pose.pose.position.z

        self.odom_x_dot = message.twist.twist.linear.x
        self.odom_y_dot = message.twist.twist.linear.y
        self.odom_z_dot = message.twist.twist.linear.z

        self.odom_roll_dot = message.twist.twist.angular.x * 180/math.pi
        self.odom_pitch_dot = message.twist.twist.angular.y * 180/math.pi
        self.odom_yaw_dot = message.twist.twist.angular.z * 180/math.pi

        self.publish_message()



    def convert_unfiltered_gps(self, message):
        """
        Sets the unfilter GPS variable
        """
        self.raw_gps = message
        self.publish_message()

    def convert_gps(self, message):
        """
        Sets the GPS variable
        """
        self.gps = message
        self.publish_message()

    def publish_message(self):
        """
        Creates and publishes the singleton message
        """
        message = RoverStateSingleton(
            map_roll = self.map_roll,
            map_pitch = self.map_pitch,
            map_yaw = self.map_yaw,
            odom_roll = self.odom_roll,
            odom_pitch = self.odom_pitch,
            odom_yaw =self.odom_yaw,
            map_x = self.map_x,
            map_y = self.map_y,
            map_z = self.map_z,
            odom_x = self.odom_x,
            odom_y = self.odom_y,
            odom_z = self.odom_z,
            map_x_dot = self.map_x_dot,
            map_y_dot = self.map_y_dot,
            map_z_dot = self.map_z_dot,
            odom_x_dot = self.odom_x_dot,
            odom_y_dot = self.odom_y_dot,
            odom_z_dot = self.odom_z_dot,
            map_roll_dot = self.map_roll_dot,
            map_pitch_dot = self.map_pitch_dot,
            map_yaw_dot = self.map_yaw_dot,
            odom_roll_dot = self.odom_roll_dot,
            odom_pitch_dot = self.odom_pitch_dot,
            odom_yaw_dot = self.odom_yaw_dot,
            gps = self.gps,
            filter_gps = self.raw_gps
        )
        self.singleton_publisher.publish(message)

def main(args=None):
    rclpy.init(args=args)
    rover_state_singleton_creator = RoverStateSingletonCreator()
    rclpy.spin(rover_state_singleton_creator)
    rover_state_singleton_creator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
