import math

from geometry_msgs.msg import TransformStamped, Vector3Stamped

import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry



def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

import math

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


def quaternion_multiply(q1, q2):
    """Multiply two quaternions."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ])


class FramePublisher(Node):

    def __init__(self):
        super().__init__('heading_tf2_frame_publisher')

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # callback function on each message
        # self.global_subscription = self.create_subscription(
        #     Imu,
        #     '/imu/data',
        #     self.handle_map_heading,
        #     1)

        self.global_yaw_sub = self.create_subscription(
            Vector3Stamped,
            '/imu/rpy/filtered',
            self.handle_rpy,
            1)

        self.local_subscription = self.create_subscription(
            Odometry,
            '/zed/odom',
            self.handle_local_heading,
            1)
        self.local_orientation = Odometry().pose.pose.orientation

        self.get_logger().info("STARTING GLOBAL CALIBRATION")
        self.i = 0
        self.average_yaw_rad = 0.0
        self.calibration_buffer_yaw = 0.0

        self.get_logger().info("STARTING LOCAL CALIBRATION")
        self.average_yaw_rad_local = 0.0
        self.calibration_buffer_yaw_local = 0.0
        self.i_local = 0

        self.create_timer(3.0, self.publish_transform)



    def quat_to_list(self,quat):
        return [quat.x, quat.y, quat.z, quat.w]

    def list_to_quat(self, list):
        quat = Imu().orientation
        quat.x = list[0]
        quat.y = list[1]
        quat.z = list[2]
        quat.w = list[3]
        return quat
    
    def handle_rpy(self, msg):

        if self.i < 100:
            self.get_logger().info(f"Calibration iteration: {self.i}")
            self.calibration_buffer_yaw += msg.vector.z
        else:
            self.average_yaw_rad = self.calibration_buffer_yaw / 100.0
        self.i += 1

    
    def handle_map_heading(self, msg):
        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'

        # Set the translation of the sensors to 0
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        # ATTEMPT BELOW TO DO WITH QUAT
        # # invert the last term to get the inverted quaternion
        # local_orientation_inv = self.local_orientation
        # local_orientation_inv.w = -1 * self.local_orientation.w

        # local_list = self.quat_to_list(local_orientation_inv)
        # gloabl_list = self.quat_to_list(msg.orientation)

        
        # # CALCULATE CHANGE BETWEEN THE TWO QUATERNIONS?
        # t.transform.rotation = self.list_to_quat(quaternion_multiply(gloabl_list, local_list))

        global_yaw = euler_from_quaternion(msg.orientation)[2]
        # self.get_logger().info(f"Global Yaw: {global_yaw}")
        # WE COULD JUST DO THE delta yaw if we want
        # global_yaw = euler_from_quaternion(msg.orientation)[2]
        # local_yaw = euler_from_quaternion(self.local_orientation)[2]
        # delta_theta = global_yaw - local_yaw
        # q = quaternion_from_euler(0, 0, msg.theta)
        # t.transform.rotation.x = q[0]
        # t.transform.rotation.y = q[1]
        # t.transform.rotation.z = q[2]
        # t.transform.rotation.w = q[3]

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)

    def publish_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        
        global_yaw = self.average_yaw_rad
        local_yaw = self.average_yaw_rad_local
        delta_theta = global_yaw - local_yaw
        q = quaternion_from_euler(0, 0, delta_theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)
    
    def handle_local_heading(self, msg):

        if self.i < 100:
            self.local_orientation = msg.pose.pose.orientation

            w = self.local_orientation.w
            x = self.local_orientation.x
            y = self.local_orientation.y
            z = self.local_orientation.z
            yaw = math.atan2(2.0 * (w*z + x*y), 1.0 - 2.0 * (y* y + z * z))
            self.get_logger().info(f"Local calibration iteration: {self.i}")
            self.calibration_buffer_yaw += yaw
            self.i += 1
        else:
            self.average_yaw_rad = self.calibration_buffer_yaw / 100.0
        # self.get_logger().info(f"Local Yaw: {yaw}")


def main():
    rclpy.init()
    node = FramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()