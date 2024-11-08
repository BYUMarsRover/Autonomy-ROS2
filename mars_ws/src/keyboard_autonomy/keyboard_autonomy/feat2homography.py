import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from rover_msgs.msg import KeyboardHomography

class Feat2HomographyNode(Node):
    '''
    :author: Nelson Durrant
    :date: November 2024

    ROS2 node that subscribes to an image topic, uses the SIFT algorithm to find keypoints and
    descriptors in both images, uses the FLANN algorithm to find matches between the descriptors, 
    and calculates the homography between the two images.

    Subscribes:
        - TODO: Add topic (sensor_msgs/msg/Image)
    Publishes:
        - /keyboard_homography (rover_msgs/msg/KeyboardHomography)
    '''

    def __init__(self):
        '''
        Creates a new Feat2Homography node.
        '''
        super().__init__('feat2homography')

        # TODO: Add topic name from camera here
        # self.subscription = self.create_subscription(Image, '<topic_name>', self.listener_callback, 10)
        '''
        Subscription to the "TODO: Add here" topic with the message type sensor_msgs/msg/Image.
        '''
        self.subscription  # Prevent unused variable warning

        self.publisher_ = self.create_publisher(KeyboardHomography, '/keyboard_homography', 10)
        '''
        Publisher to the "/keyboard_homography" topic with the message type KeyboardHomography.
        '''

        self.bridge = CvBridge()
        keyboard_img = cv2.imread("../images/keyboard_cropped.jpg")

    def listener_callback(self, msg):
        '''
        Callback function for the "TODO: Add here" subscription.
        Uses the SIFT and FLANN algorithms to find the homography between the keyboard image and the
        received image.

        :param msg: The Image message received from the "TODO: Add here" topic.
        '''
        # Convert ROS Image to OpenCV
        camera_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # TODO: Add SIFT algorithm to find keypoints and descriptors in both images
        # TODO: Add FLANN algorithm to find matches between the descriptors
        # TODO: Add homography calculation (Should be a 3x3 matrix)

        keyboard_homography = KeyboardHomography()
        keyboard_homography.homography = None

        self.publisher_.publish(keyboard_homography)

def main(args=None):
    rclpy.init(args=args)
    node = Feat2HomographyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()