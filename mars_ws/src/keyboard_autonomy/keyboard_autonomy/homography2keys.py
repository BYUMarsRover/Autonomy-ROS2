import rclpy
from rclpy.node import Node
import keyboard_calibration
from rover_msgs.msg import KeyboardHomography, KeyLocations

class Homography2KeysNode(Node):
    '''
    :author: Nelson Durrant
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

        self.publisher_ = self.create_publisher(KeyLocations, '/key_locations', 10)
        '''
        Publisher to the "/key_locations" topic with the message type KeyLocations.
        '''

    def listener_callback(self, msg):
        '''
        Callback function for the "/keyboard_homography" subscription.
        Uses the homography to find the key locations in the camera image frame.

        :param msg: The KeyboardHomography message received from the "/keyboard_homography" topic.
        '''

        # TODO: Add key locations calculation

        key_locations = KeyLocations()

        self.publisher_.publish(key_locations)

def main(args=None):
    rclpy.init(args=args)
    node = Homography2KeysNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()