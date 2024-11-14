import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from keyboard_calibration import KEYPIX
from rover_msgs.msg import KeyboardHomography, KeyLocations


class Homography2KeysNode(Node):
    '''
    :author: Nelson Durrant, Chloe Hilton
    :date: November 2024

    ROS2 node that uses the homography between the keyboard image and the camera image to find the key
    locations in the camera image frame.

    Subscribes:
        - /keyboard_homography (rovers_msgs/msg/KeyboardHomography)
    Publishes:
        - /key_locations (rovers_msgs/msg/KeyLocations)
    '''

    def __init__(self):
        '''
        Creates a new Homography2Keys node.
        '''
        super().__init__('homography2keys')

        self.subscription = self.create_subscription(KeyboardHomography, '/keyboard_homography', self.listener_callback, 10)
        '''
        Subscription to the "/keyboard_homography" topic with the message type KeyboardHomography.
        '''
        self.subscription  # Prevent unused variable warning

        self.publisher = self.create_publisher(KeyLocations, '/key_locations', 10)
        '''
        Publisher to the "/key_locations" topic with the message type KeyLocations.
        '''

        self.get_logger().info("Homography2KeysNode started")

    def listener_callback(self, msg):
        '''
        Callback function for the "/keyboard_homography" subscription.
        Uses the homography to find the key locations in the camera image frame.

        :param msg: The KeyboardHomography message received from the "/keyboard_homography" topic.
        '''
        homography = np.array(msg.homography).reshape((3, 3))
        key_points = np.array(list(KEYPIX.values()), dtype=np.float32).reshape(-1, 1, 2)
        key_locations = KeyLocations()
        transformed_points = cv2.perspectiveTransform(key_points, homography)
        for key, point in zip(KEYPIX.keys(), transformed_points):
            setattr(key_locations, key, point.flatten().tolist())

        self.publisher.publish(key_locations)


def main(args=None):
    rclpy.init(args=args)
    node = Homography2KeysNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
