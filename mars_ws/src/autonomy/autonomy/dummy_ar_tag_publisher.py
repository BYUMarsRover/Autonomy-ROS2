########

# This script is used to publish dummy AR tag data to the /aruco_detect_logi/fiducial_transforms topic
# This is useful for testing the AR tag detection and aruco navigation when run in tandem with the state machine
# Written by Gabe Slade Jan 2025


import rclpy
from rclpy.node import Node
from rover_msgs.msg import FiducialTransformArray, FiducialTransform
# from geometry_msgs.msg import Transform
import numpy as np

class dummyARTagPublisher(Node):
    def __init__(self):
        super().__init__('dummy_AR_tag_publisher')

        self.publisher_ = self.create_publisher(FiducialTransformArray, '/aruco_detect_logi/fiducial_transforms', 10)
        timer_period = 0.2  # seconds - 5 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.x = 3.0
        self.y = -0.8
        self.z = 2.0

    def arc_increment(self): # make the object move side to side in an arc around the usb camera
        self.z = 3*np.cos(3/2*np.pi + np.pi*np.abs(np.sin(np.pi*self.i/100)))
        self.x = -3*np.sin(3/2*np.pi + np.pi*np.abs(np.sin(np.pi*self.i/100)))

    def linear_increment(self): # make the object oscillate linearly in the z direction ()
        self.x = 0.0
        self.z = np.cos(self.i/100)*5

    def timer_callback(self):
        msg = FiducialTransformArray()
        transform_msg = FiducialTransform()
        transform_msg.fiducial_id = int(1)
        transform_msg.transform.translation.x = self.x
        transform_msg.transform.translation.y = self.y
        transform_msg.transform.translation.z = self.z

        msg.transforms.append(transform_msg)
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'x:{self.x:.3f}, y:{self.y:.3f}, z:{self.z:.3f}')
        self.i += 1
        # self.arc_increment()
        self.linear_increment()


def main(args=None):
    rclpy.init(args=args)

    dummy_AR_tag_publisher = dummyARTagPublisher()

    rclpy.spin(dummy_AR_tag_publisher)

    dummy_AR_tag_publisher.destroy_node()
    rclpy.shutdown()