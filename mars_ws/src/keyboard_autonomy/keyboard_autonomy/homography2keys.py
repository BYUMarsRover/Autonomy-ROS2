import rclpy
from rclpy.node import Node
import keyboard_calibration
# TODO: Add custom homography message (with timestamp)
# TODO: Add custom key locations message (with timestamp)

class Homography2KeysNode(Node):
    '''
    :author: Nelson Durrant
    :date: November 2024

    ROS2 node that uses the homography between the keyboard image and the camera image to find the key
    locations in the camera image frame.

    Subscribes:
        - /keyboard_homography (TODO: Add custom message)
    Publishes:
        - /key_locations (TODO: Add custom message)
    '''

    def __init__(self):
        '''
        Creates a new Homography2Keys node.
        '''
        super().__init__('homography2keys')

        # TODO: Add custom homography message (with timestamp) to subscriber
        # self.subscription = self.create_subscription(<custom_message>, '/keyboard_homography', self.listener_callback, 10)
        '''
        Subscription to the "/keyboard_homography" topic with the message type TODO: Add here.
        '''
        self.subscription  # Prevent unused variable warning

        # TODO: Add custom key locations message (with timestamp) to publisher
        # self.publisher_ = self.create_publisher(<custom_message>, '/key_locations', 10)
        '''
        Publisher to the "/key_locations" topic with the message type TODO: Add here.
        '''

    def listener_callback(self, msg):
        '''
        Callback function for the "/keyboard_homography" subscription.
        Uses the homography to find the key locations in the camera image frame.

        :param msg: The TODO: Add here message received from the "/keyboard_homography" topic.
        '''

        # TODO: Add key locations calculation
        # TODO: Publish key locations as custom message

def main(args=None):
    rclpy.init(args=args)
    node = Homography2KeysNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()