#!/usr/bin/python3

import math
import rclpy
from rclpy.node import Node
from fiducial_msgs.msg import FiducialTransformArray
from rover_msgs.msg import FiducialData

class FiducialDataNode(Node):
    """
    This class publishes information on spotted AR tags.
    """
    def __init__(self):
        super().__init__('fiducial_data_node')

        # Subscribing to fiducial detection topics from ZED and Logi cameras
        self.sub_ar_detection_zed = self.create_subscription(
            FiducialTransformArray,
            '/aruco_detect_zed/fiducial_transforms',
            self.arTagCallback,
            10)
        
        self.sub_ar_detection_logi = self.create_subscription(
            FiducialTransformArray,
            '/aruco_detect_logi/fiducial_transforms',
            self.arTagCallback,
            10)

        # Publisher for processed fiducial data
        self.fiducial_data_publisher = self.create_publisher(FiducialData, '/fiducial_data', 10)

        # Initialize variables
        self.distance = None
        self.angle_offset = None
        self.num_tags = None

    def publish_msg(self):
        # Create and publish FiducialData message
        message = FiducialData(
            num_fiducials=self.num_tags,
            dist_to_fiducial=self.distance,
            angle_offset=self.angle_offset
        )
        self.fiducial_data_publisher.publish(message)

    def arTagCallback(self, msg):
        """
        Called when the rover receives a new message from the /fiducial_transforms publisher
        and updates the State's ArTagMessage.
        """
        try:
            self.num_tags = len(msg.transforms)
            if self.num_tags > 0:
                # Calculate distance and angle to tag from the Rover's current location
                x = msg.transforms[0].transform.translation.x
                z = msg.transforms[0].transform.translation.z
                self.angle_offset = math.degrees(math.atan2(x, z))
                self.distance = math.sqrt(x * x + z * z)

                # Publish the processed message
                self.publish_msg()
        except Exception as e:
            self.get_logger().error(f'Error processing ArTag message: {e}')


def main(args=None):
    # Initialize ROS 2 Python client library
    rclpy.init(args=args)

    # Create instance of the FiducialDataNode
    fiducial_data_node = FiducialDataNode()

    # Spin the node to keep it active
    rclpy.spin(fiducial_data_node)

    # Cleanup
    fiducial_data_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
