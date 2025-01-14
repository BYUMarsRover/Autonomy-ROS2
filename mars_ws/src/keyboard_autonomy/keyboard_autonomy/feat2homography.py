import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
from rover_msgs.msg import KeyboardHomography
import time

MIN_MATCH_COUNT = 10


class Feat2HomographyNode(Node):
    '''
    :author: Nelson Durrant, Hannah Spigarelli
    :date: November 2024

    ROS2 node that subscribes to an image topic, uses the SIFT algorithm to find keypoints and
    descriptors in both images, uses the FLANN algorithm to find matches between the descriptors, 
    and calculates the homography between the two images.

    Subscribes:
        - /image_raw (sensor_msgs/msg/Image)
    Publishes:
        - /keyboard_homography (rover_msgs/msg/KeyboardHomography)
    '''

    def __init__(self):
        '''
        Creates a new Feat2Homography node.
        '''
        super().__init__('feat2homography')

        self.subscription = self.create_subscription(Image, '/image_raw', self.listener_callback, 10)
        '''
        Subscription to the "/image_raw" topic with the message type sensor_msgs/msg/Image.
        '''
        self.subscription  # Prevent unused variable warning

        self.publisher = self.create_publisher(KeyboardHomography, '/keyboard_homography', 10)
        '''
        Publisher to the "/keyboard_homography" topic with the message type KeyboardHomography.
        '''

        self.bridge = CvBridge()
        self.keyboard_img = cv2.imread("/home/marsrover/mars_ws/src/keyboard_autonomy/images/2024-12-03-085842.jpg")

        self.get_logger().info("Feat2HomographyNode started")

    def listener_callback(self, msg):
        '''
        Callback function for the "/image_raw" subscription.
        Uses the SIFT and FLANN algorithms to find the homography between the keyboard image and the
        received image.

        :param msg: The Image message received from the Image topic.
        '''
        # Convert ROS Image to OpenCV
        camera_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # SIFT and FLANN
        # Initiate SIFT detector and find the keypoints and descriptors
        sift = cv2.SIFT_create(nfeatures=500, nOctaveLayers=2, contrastThreshold=0.04, edgeThreshold=5)
        kp1, des1 = sift.detectAndCompute(self.keyboard_img, None)
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
        M = None
        # If enough matches are found, calculate the homography
        if len(good_matches) >= MIN_MATCH_COUNT:

            # Get the key points from the good matches
            src_pts = np.float32([kp1[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
            dst_pts = np.float32([kp2[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)

            # Calculate the homography matrix using RANSAC
            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)

        else:
            self.get_logger().warn("Insufficient # of matches found - %d/%d" % (len(good_matches), MIN_MATCH_COUNT))

        matchesMask = mask.ravel().tolist()

        # TODO: get rid of the drawn outline when this node is finished
        h, w = self.keyboard_img.shape[0:2]
        pts = np.float32([[0, 0], [0, h - 1], [w - 1, h - 1], [w - 1, 0]]).reshape(-1, 1, 2)  # get the corners
        outline = cv2.perspectiveTransform(pts, M)  # transform the corners of the first image onto the second
        img2 = cv2.polylines(camera_image, [np.int32(outline)], True, 255, 3, cv2.LINE_AA)  # draw the box

        draw_params = dict(
            singlePointColor=None,
            matchesMask=matchesMask,
            flags=2)
        img3 = cv2.drawMatches(self.keyboard_img, kp1, img2, kp2, good_matches, None, **draw_params)
        cv2.imwrite("matches.jpg", img3)
        keyboard_homography = KeyboardHomography()
        keyboard_homography.homography = M.flatten().tolist()

        self.publisher.publish(keyboard_homography)


def main(args=None):
    rclpy.init(args=args)
    node = Feat2HomographyNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
