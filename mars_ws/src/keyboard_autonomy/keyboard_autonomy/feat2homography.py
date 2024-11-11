import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
from rover_msgs.msg import KeyboardHomography

MIN_MATCH_COUNT = 10


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
        self.keyboard_img = cv2.imread("../images/keyboard_cropped.jpg")

    def listener_callback(self, msg):
        '''
        Callback function for the "TODO: Add here" subscription.
        Uses the SIFT and FLANN algorithms to find the homography between the keyboard image and the
        received image.

        :param msg: The Image message received from the "TODO: Add here" topic.
        '''
        # Convert ROS Image to OpenCV
        camera_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Initiate SIFT detector and find the keypoints and descriptors
        sift = cv2.SIFT_create(nfeatures=500, nOctaveLayers=2, contrastThreshold=0.04, edgeThreshold=10)
        kp1, des1 = sift.detectAndCompute(self.keyboard_image, None)
        kp2, des2 = sift.detectAndCompute(camera_image, None)

        # Set FLANN parameters
        FLANN_INDEX_KDTREE = 0
        index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
        search_params = dict(checks=50)

        # Create the FLANN matcher and find matches (best two matches for each descriptor)
        flann = cv2.FlannBasedMatcher(index_params, search_params)
        matches = flann.knnMatch(des1, des2, k=2)

        # Store all the good matches using Lowe's ratio test
        # Check to make sure that the distance to the closest match is sufficiently
        # less than the distance to the second-closest match
        good_matches = []
        for m, n in matches:
            if m.distance < 0.7 * n.distance:
                good_matches.append(m)

        # If enough matches are found, calculate the homography
        if len(good_matches) >= MIN_MATCH_COUNT:

            # Get the keypoints from the good matches
            src_pts = np.float32([kp1[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
            dst_pts = np.float32([kp2[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)

            # Calculate the homography matrix using RANSAC
            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)

        else:
            self.get_logger().warn("Insufficient # of matches found - %d/%d" % (len(good_matches), MIN_MATCH_COUNT))

        keyboard_homography = KeyboardHomography()
        keyboard_homography.header.stamp = msg.header.stamp
        keyboard_homography.homography = M.flatten().tolist()  # TODO: Test this

        self.publisher_.publish(keyboard_homography)


def main(args=None):
    rclpy.init(args=args)
    node = Feat2HomographyNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
