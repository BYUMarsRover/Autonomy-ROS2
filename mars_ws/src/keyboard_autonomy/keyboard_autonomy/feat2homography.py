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
        self.keyboard_img = cv2.imread("/home/marsrover/mars_ws/src/keyboard_autonomy/images/keyboard_better.jpg")

        self.get_logger().info("Feat2HomographyNode started")

    def listener_callback(self, msg):
        '''
        Callback function for the "/image_raw" subscription.
        Uses the SIFT and FLANN algorithms to find the homography between the keyboard image and the
        received image.

        :param msg: The Image message received from the "TODO: Add here" topic.
        '''
        # Convert ROS Image to OpenCV
        camera_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # FISHEYE CONVERSION
        h, w = camera_image.shape[:2]

        # Camera matrix (K) from calibration
        K = np.array([[240.0742270720785, 0.0, 303.87271958823317], [0.0, 240.2956790772891, 242.63742883611513],
                      [0.0, 0.0, 1.0]], dtype=np.float32)

        # Distortion coefficients (D) from calibration
        D = np.array([[-0.032316597779098725],
                      [-0.014192392170667197],
                      [0.004485507618487958],
                      [-0.0007679973472430706]], dtype=np.float32)

        # Adjust K to prevent lots of zooming
        scaled_K = K.copy()
        scaled_K[0, 0] *= 0.8  # Reduce focal length in x-direction
        scaled_K[1, 1] *= 0.8  # Reduce focal length in y-direction

        # Undistortion map
        map1, map2 = cv2.fisheye.initUndistortRectifyMap(
            K, D, np.eye(3), scaled_K, (w, h), cv2.CV_16SC2
        )

        # Undistort the image
        undistorted_img = cv2.remap(camera_image, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

        # Show and save the undistorted image TODO take this out after testing
        cv2.imshow("Original", camera_image)
        cv2.imshow("Undistorted", undistorted_img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        cv2.imwrite("undistorted_img.png", undistorted_img)

        # SIFT and FLANN
        # Initiate SIFT detector and find the keypoints and descriptors
        sift = cv2.SIFT_create(nfeatures=500, nOctaveLayers=2, contrastThreshold=0.04, edgeThreshold=10)
        kp1, des1 = sift.detectAndCompute(self.keyboard_image, None)
        kp2, des2 = sift.detectAndCompute(undistorted_img, None)

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
