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
        self.subscription = self.create_subscription(Image, 'test', self.listener_callback, 10)
        '''
        Subscription to the "TODO: Add here" topic with the message type sensor_msgs/msg/Image.
        '''
        self.subscription  # Prevent unused variable warning

        self.publisher_ = self.create_publisher(KeyboardHomography, '/keyboard_homography', 10)
        '''
        Publisher to the "/keyboard_homography" topic with the message type KeyboardHomography.
        '''
        self.get_logger().info("At the init function")

        self.bridge = CvBridge()
        # self.keyboard_img = cv2.imread("../images/keyboard_cropped.jpg")
        start_time = time.time()

        # Load the images using OpenCV
        img1 = cv2.imread("/home/marsrover/mars_ws/src/keyboard_autonomy/images/keyboard_better.jpg")
        img2 = cv2.imread("/home/marsrover/mars_ws/src/keyboard_autonomy/images/keyboard1.jpg")

        image_read_time = time.time()
        sift_detect_and_compute_time = time.time()

        orb = cv2.ORB_create(600)
        kp1, des1 = orb.detectAndCompute(img1, None)
        kp2, des2 = orb.detectAndCompute(img2, None)
        orb_detect_and_compute_time = time.time()

        # Set FLANN parameters (TODO: What are these?)
        FLANN_INDEX_KDTREE = 0
        FLANN_INDEX_LSH = 6
        index_params = dict(algorithm=FLANN_INDEX_LSH, table_number=6)  # key_size = 12, multi_probe_level = 1) #2
        # index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
        search_params = dict(checks=50)

        parameter_dictionary_time = time.time()

        # Create the FLANN matcher and find matches (best two matches for each descriptor)
        flann = cv2.FlannBasedMatcher(index_params, search_params)
        matches = flann.knnMatch(des1, des2, k=2)

        matches_calculated_time = time.time()

        # Store all the good matches using Lowe's ratio test
        # Check to make sure that the distance to the closest match is sufficiently
        # less than the distance to the second-closest match
        good_matches = []
        skipped_matches = 0
        for match_pair in matches:
            if len(match_pair) == 2:  # Ensure we have two matches
                m, n = match_pair
                if m.distance < 0.7 * n.distance:
                    good_matches.append(m)
            else:
                skipped_matches += 1

        # If enough matches are found, calculate the homography
        if len(good_matches) >= MIN_MATCH_COUNT:
            print(f'Found: {len(good_matches)} good matches out of {len(matches)} total matches')
            print(f'skipped: {skipped_matches} because there weren\'t two pairs')

            # Get the keypoints from the good matches
            src_pts = np.float32([kp1[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
            dst_pts = np.float32([kp2[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)

            # Calculate the homography matrix and mask
            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
            matchesMask = mask.ravel().tolist()

            homography_calculated_time = time.time()
            # Plot and display the results
            h, w = img1.shape[0:2]
            keyboard_homography = KeyboardHomography()
            keyboard_homography.homography = M.flatten().tolist()  # TODO: Test this
            self.get_logger().info("Keyboard homography")
            self.get_logger().info(str(keyboard_homography))
            self.publisher_.publish(keyboard_homography)

        else:
            print("Not enough matches are found - %d/%d" % (len(good_matches), MIN_MATCH_COUNT))

        print(f"time for image reading: {image_read_time - start_time}")
        print(
            f"time for detecting and computing SIFT keypoints and descriptors: {sift_detect_and_compute_time - image_read_time}")
        print(
            f"time for detecting and computing ORB keypoints and descriptors: {orb_detect_and_compute_time - sift_detect_and_compute_time}")
        print(f"time for creating the dictionaries: {parameter_dictionary_time - orb_detect_and_compute_time}")
        print(f"time for calling FlannBasedMatcher and knnMatch: {matches_calculated_time - parameter_dictionary_time}")
        print(
            f"time for calculating the homography matrix and mask: {homography_calculated_time - matches_calculated_time}")


    def listener_callback(self, msg):
        '''
        Callback function for the "TODO: Add here" subscription.
        Uses the SIFT and FLANN algorithms to find the homography between the keyboard image and the
        received image.

        :param msg: The Image message received from the "TODO: Add here" topic.
        '''
        # Convert ROS Image to OpenCV
        #camera_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        start_time = time.time()

        # Load the images using OpenCV
        img1 = cv2.imread("../images/keyboard_better.jpg")
        img2 = cv2.imread("../images/keyboard1.jpg")

        image_read_time = time.time()
        sift_detect_and_compute_time = time.time()

        orb = cv2.ORB_create(600)
        kp1, des1 = orb.detectAndCompute(img1, None)
        kp2, des2 = orb.detectAndCompute(img2, None)
        orb_detect_and_compute_time = time.time()

        # Set FLANN parameters (TODO: What are these?)
        FLANN_INDEX_KDTREE = 0
        FLANN_INDEX_LSH = 6
        index_params = dict(algorithm=FLANN_INDEX_LSH, table_number=6)  # key_size = 12, multi_probe_level = 1) #2
        # index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
        search_params = dict(checks=50)

        parameter_dictionary_time = time.time()

        # Create the FLANN matcher and find matches (best two matches for each descriptor)
        flann = cv2.FlannBasedMatcher(index_params, search_params)
        matches = flann.knnMatch(des1, des2, k=2)

        matches_calculated_time = time.time()

        # Store all the good matches using Lowe's ratio test
        # Check to make sure that the distance to the closest match is sufficiently
        # less than the distance to the second closest match
        good_matches = []
        skipped_matches = 0
        for match_pair in matches:
            if len(match_pair) == 2:  # Ensure we have two matches
                m, n = match_pair
                if m.distance < 0.7 * n.distance:
                    good_matches.append(m)
            else:
                skipped_matches += 1

        # If enough matches are found, calculate the homography
        if len(good_matches) >= MIN_MATCH_COUNT:
            print(f'Found: {len(good_matches)} good matches out of {len(matches)} total matches')
            print(f'skipped: {skipped_matches} because there weren\'t two pairs')

            # Get the keypoints from the good matches
            src_pts = np.float32([kp1[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
            dst_pts = np.float32([kp2[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)

            # Calculate the homography matrix and mask
            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
            matchesMask = mask.ravel().tolist()

            homography_calculated_time = time.time()
            # Plot and display the results
            h, w = img1.shape[0:2]
            keyboard_homography = KeyboardHomography()
            keyboard_homography.header.stamp = msg.header.stamp
            keyboard_homography.homography = M.flatten().tolist()  # TODO: Test this
            self.get_logger().info("Keyboard homography")
            self.get_logger().info(str(keyboard_homography))
            self.publisher_.publish(keyboard_homography)

        else:
            print("Not enough matches are found - %d/%d" % (len(good_matches), MIN_MATCH_COUNT))

        print(f"time for image reading: {image_read_time - start_time}")
        print(
            f"time for detecting and computing SIFT keypoints and descriptors: {sift_detect_and_compute_time - image_read_time}")
        print(
            f"time for detecting and computing ORB keypoints and descriptors: {orb_detect_and_compute_time - sift_detect_and_compute_time}")
        print(f"time for creating the dictionaries: {parameter_dictionary_time - orb_detect_and_compute_time}")
        print(f"time for calling FlannBasedMatcher and knnMatch: {matches_calculated_time - parameter_dictionary_time}")
        print(
            f"time for calculating the homography matrix and mask: {homography_calculated_time - matches_calculated_time}")


def main(args=None):
    rclpy.init(args=args)
    node = Feat2HomographyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()