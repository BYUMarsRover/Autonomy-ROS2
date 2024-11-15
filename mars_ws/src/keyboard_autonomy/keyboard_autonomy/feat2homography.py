import rclpy
from matplotlib import pyplot as plt
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

        self.publisher = self.create_publisher(KeyboardHomography, '/keyboard_homography', 10)
        '''
        Publisher to the "/keyboard_homography" topic with the message type KeyboardHomography.
        '''
        self.get_logger().info("At the init function")

        self.bridge = CvBridge()
        # self.keyboard_img = cv2.imread("../images/keyboard_cropped.jpg")
        start_time = time.time()

        elapsed_100 = self.orb_calculate("/home/marsrover/mars_ws/src/keyboard_autonomy/images/keyboard_better_rotated.jpg", "/home/marsrover/mars_ws/src/keyboard_autonomy/images/keyboard_better.jpg",
                                    'key_board_better_inverted_compare.png', orb_create_matches=100)
        elapsed_2000 = self.orb_calculate("/home/marsrover/mars_ws/src/keyboard_autonomy/images/keyboard_better_rotated.jpg", "/home/marsrover/mars_ws/src/keyboard_autonomy/images/keyboard_better.jpg",
                                     'key_board_better_inverted_compare.png', orb_create_matches=2000)
        elapsed_4000 = self.orb_calculate("/home/marsrover/mars_ws/src/keyboard_autonomy/images/keyboard_better_rotated.jpg", "/home/marsrover/mars_ws/src/keyboard_autonomy/images/keyboard_better.jpg",
                                     'key_board_better_inverted_compare.png', orb_create_matches=4000)
        elapsed_6000 = self.orb_calculate("/home/marsrover/mars_ws/src/keyboard_autonomy/images/keyboard_better_rotated.jpg", "/home/marsrover/mars_ws/src/keyboard_autonomy/images/keyboard_better.jpg",
                                     'key_board_better_inverted_compare.png', orb_create_matches=6000)
        elapsed_8000 = self.orb_calculate("/home/marsrover/mars_ws/src/keyboard_autonomy/keyboard_better_rotated.jpg", "/home/marsrover/mars_ws/src/keyboard_autonomy/images/keyboard_better.jpg",
                                     'key_board_better_inverted_compare.png', orb_create_matches=8000)


    def listener_callback(self, msg):
        '''
        Callback function for the "TODO: Add here" subscription.
        Uses the SIFT and FLANN algorithms to find the homography between the keyboard image and the
        received image.

        :param msg: The Image message received from the "TODO: Add here" topic.
        '''
        # Convert ROS Image to OpenCV
        camera_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        elapsed_100 = self.orb_calculate("../images/keyboard_better_rotated.jpg", camera_image,
                                    'key_board_better_inverted_compare.png', orb_create_matches=500)

    def sift_calculate(self, first_image, second_image, path_to_save_result, orb_create_matches=300, min_match_count=10,
                       flann_index_lsh=6):
        start_time = time.time()

        # Minimum number of matches required
        MIN_MATCH_COUNT = min_match_count

        # Load the images using OpenCV
        img1 = cv2.imread(first_image)
        img2 = cv2.imread(second_image)
        image_read_time = time.time()

        # Initiate SIFT detector and find the keypoints and descriptors
        sift = cv2.SIFT_create(nfeatures=500, nOctaveLayers=2, contrastThreshold=0.04, edgeThreshold=10)
        kp1, des1 = sift.detectAndCompute(img1, None)
        kp2, des2 = sift.detectAndCompute(img2, None)
        sift_detect_and_compute_time = time.time()

        # Set FLANN parameters
        FLANN_INDEX_KDTREE = 0
        index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
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
        for m, n in matches:
            if m.distance < 0.7 * n.distance:
                good_matches.append(m)

        # If enough matches are found, calculate the homography
        homography_calculated_time = 0
        if len(good_matches) >= MIN_MATCH_COUNT:
            print(f'Found: {len(good_matches)} good matches out of {len(matches)} total matches')

            # Get the keypoints from the good matches
            src_pts = np.float32([kp1[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
            dst_pts = np.float32([kp2[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)

            # Calculate the homography matrix and mask
            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
            matchesMask = mask.ravel().tolist()

            homography_calculated_time = time.time()
            # Plot and display the results
            h, w = img1.shape[0:2]
            pts = np.float32([[0, 0], [0, h - 1], [w - 1, h - 1], [w - 1, 0]]).reshape(-1, 1, 2)  # get the corners
            outline = cv2.perspectiveTransform(pts, M)  # transform the corners of the first image onto the second
            img2 = cv2.polylines(img2, [np.int32(outline)], True, 255, 20, cv2.LINE_AA)  # draw the box
            draw_params = dict(
                singlePointColor=None,
                matchesMask=matchesMask,
                flags=2,
                matchesThickness=20  # Adjust this value to change the line thickness between matched points
            )
            img3 = cv2.drawMatches(img1, kp1, img2, kp2, good_matches, None, **draw_params)
            plt.imshow(img3, 'gray')
            plt.savefig(path_to_save_result)
            # plt.show()

        else:
            print("Not enough matches are found - %d/%d" % (len(good_matches), MIN_MATCH_COUNT))

        end_time = time.time()

        elapsed_time = end_time - start_time

        print(f"time for image reading: {image_read_time - start_time}")
        print(
            f"time for detecting and computing SIFT keypoints and descriptors: {sift_detect_and_compute_time - image_read_time}")
        print(f"time for creating the dictionaries: {parameter_dictionary_time - sift_detect_and_compute_time}")
        print(f"time for calling FlannBasedMatcher and knnMatch: {matches_calculated_time - parameter_dictionary_time}")
        print(
            f"time for calculating the homography matrix and mask: {homography_calculated_time - matches_calculated_time}")


    def orb_calculate(self, first_image, second_image, path_to_save_result, orb_create_matches=300, min_match_count=10,
                      flann_index_lsh=6):
        start_time = time.time()

        # Minimum number of matches required
        MIN_MATCH_COUNT = min_match_count

        # Load the images using OpenCV
        img1 = cv2.imread(first_image)
        img2 = cv2.imread(second_image)
        image_read_time = time.time()

        # Do the orb create and detect
        orb = cv2.ORB_create(orb_create_matches)
        kp1, des1 = orb.detectAndCompute(img1, None)
        kp2, des2 = orb.detectAndCompute(img2, None)
        orb_detect_and_compute_time = time.time()

        # Set FLANN parameters
        FLANN_INDEX_LSH = flann_index_lsh
        index_params = dict(algorithm=FLANN_INDEX_LSH, table_number=6)
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

        homography_calculated_time = 0
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
            pts = np.float32([[0, 0], [0, h - 1], [w - 1, h - 1], [w - 1, 0]]).reshape(-1, 1, 2)  # get the corners
            outline = cv2.perspectiveTransform(pts, M)  # transform the corners of the first image onto the second
            img2 = cv2.polylines(img2, [np.int32(outline)], True, 255, 20, cv2.LINE_AA)  # draw the box
            draw_params = dict(
                singlePointColor=None,
                matchesMask=matchesMask,
                flags=2,
                matchesThickness=20  # Adjust this value to change the line thickness between matched points
            )
            img3 = cv2.drawMatches(img1, kp1, img2, kp2, good_matches, None, **draw_params)
            plt.imshow(img3, 'gray')
            plt.savefig(path_to_save_result)
            # showing the plot currently doesn't work
            # plt.show()

        else:
            print("Not enough matches are found - %d/%d" % (len(good_matches), MIN_MATCH_COUNT))

        end_time = time.time()
        elapsed_time = end_time - start_time
        print(f"time for image reading: {image_read_time - start_time}")
        print(
            f"time for detecting and computing ORB keypoints and descriptors: {orb_detect_and_compute_time - image_read_time}")
        print(f"time for creating the dictionaries: {parameter_dictionary_time - orb_detect_and_compute_time}")
        print(f"time for calling FlannBasedMatcher and knnMatch: {matches_calculated_time - parameter_dictionary_time}")
        print(
            f"time for calculating the homography matrix and mask: {homography_calculated_time - matches_calculated_time}")
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