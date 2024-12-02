#!/usr/bin/env python3

# General flow:
# 1. Get depth image from ZED SDK
# 2. Calculate normal map from depth image
# 3. Slice that normal map into 3 based on the depth
# 4. Use blob detection (another way to say obstacle detection) in each of those slices
# 5. Combine close objects in each of those slices
# 6. Publish object positions

# If you want to tune the sensitivity of this algorithm, start with normFactor,
# curveFactor, and the params.min_area parameter in the initialize_blob_detect function.
# You can also adjust the skip factor in the launch file. This will also increase/decrease
# compute requirements.
# Tuning seems to be a balance between detecting object that appear small in the image
# and not detecting the ground.

# from gotogoal import GoToGoal # <-- what's this?

# TODO: bushes, poles (such as aruco tag poles)

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import cv2

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String

# Assuming these custom messages are also converted to ROS2
from rover_msgs.msg import ObstacleInfo, WaypointNav, NavStatus, ZedObstacles

class ObstacleDetect(Node):
    def __init__(self):
        super().__init__('obstacle_detect')

        # Constants
        self.alpha = 0.3  # constant for low pass filter
        self.ROVER_CLEARANCE = 5  # [m]
        self.RANGE_HIGH = 5.0  # [m] max distance at which rplidar will detect objects
        self.RANGE_LOW = 0.15  # [m] min distance at which rplidar will detect objects
        self.bridge = CvBridge()
        self.LOCATION_ARRAY_SIZE = 100  # size of array of prev GPS points for stuck
        self.STUCK_RADIUS = (
            1  # [m] stuck if median of location array is within this radius
        )
        self.CAMERA_V_FOV = np.deg2rad(70)
        self.execute_period = 0.1

        # Variables
        self.center_obs = False  # flag for center obstacle
        self.goal_obs = False  # flag for obstacle in goal direction
        self.obstacle_centroid = 0
        self.weight = 0.0
        self.heading_offset = [0, 0]  # [rad, deg]
        self.is_stuck = False

        self.points = np.zeros((90, 2))  # [distances (m), angle (deg)]
        self.points[:, 0] = np.inf
        self.points[:, 1] = np.arange(45, -45, -1)

        self.locations = np.zeros((self.LOCATION_ARRAY_SIZE, 2))
        self.locations[:, 0] = np.arange(self.LOCATION_ARRAY_SIZE)
        self.north = 0
        self.east = 0
        self.v = 0

        self.gimbal_tilt = 0 # is this a variable of degrees or radians?

        self.prev_scores = np.zeros(
            6
        )  # is_wall, center_obs, goal_obs, obstacle_centroid, weight
        self.obs = np.zeros((0, 0))  # fully filtered data points

        # state flags
        self.msgread = {"image": False, "depth": False}
        self.init = False
        self.shutdown = False
        self.ready = False

        # images
        self.img = 0  # Stores image
        self.depth_img = 0  # Stores depth map
        self.gry_depth = 0  # Stores grayscale of depth map
        self.gry_depth_shape = (0, 0, 0)
        self.norm_img = (
            0  # Stores surface normals (rgb image with components as colors)
        )
        self.dir_img = 0  # Stores grayscale of z-direction surface normal
        self.slope_img = 0  # Stores gradient only in y-direction
        self.curve_img = 0  # Stores image of magnitude of curvature
        self.composite_img = (
            0  # Stores composite image after partitioning and combining
        )

        self.keyPoints = 0  # Stores obstacle locations from blob detector
        self.keyLoc = 0
        self.keyMeas = 0
        self.close_slope = 0

        # ROS2 parameters
        self.declare_parameter('zed_type', 'zed2')
        self.declare_parameter('skip', 5)
        self.declare_parameter('display', False)

        self.zed_type = self.get_parameter('zed_type').value
        self.skip = self.get_parameter('skip').value
        self.display = self.get_parameter('display').value

        self.get_logger().info(f"zed_type: {self.zed_type}")

        # Obstacle detection parameters
        
        ### TESTING - What parameters do what? 
        ###

        self.groundDistCutoff = 0.6  # This is the distance in meters below the camera at which anything
            # in the depth field will be ignored.

        self.groundAngleCutoff = np.deg2rad(2.0)  # This is the angle at which the cutoff plane defined by
            # groundDistCutoff wil be tilted up, increasing the cutoff height at further distances.
            # Needs to be greater than or equal to 0. Greater than 5 degrees is not recommended.
        
        self.minDist = (
            0.5  # Minimum camera distance (~0.75m is closest zed will reliably measure)
        )

        # NOTE - increasing this didn't do anything to increase the distance that the trashcan was detected.
        # Tried 20
        self.maxDist = ( 
            3.5  # Maximum camera distance (~20m is farthest zed will measure)
        )

        # NOTE - increasing only marginally increased distance that the trashcan was detected. By about 0.25 m
        # Changing it to 1 created a runtime error.
        # Tried 10 & 1
        self.distFrames = (
            2  # Chop distance into discrete steps of this size to detect obstacles
        )
        
        # NOTE - not much benefit to detection distance when changing it to 2 and 5
        # This seems to make the colors in the distance image brighter. Maybe it's adding frames together?
        self.overlap = 1  # Define how much the discrete steps should overlap

        # NOTE - not sure what this does. Something to do with surface NORMALS
        # The lower it is, the more forgiving the surface normals are in detection.

        self.dirThreshold = 0.05
        #self.dirThreshold = 0.05
        
        # NOTE - not sure what this does. Something to do with surface NORMALS
        # Increasing this (tried 0.5) made the obstacle detection very forgiving - to a fault. It began to label the ground as an obstacle.
        # However, this could be due to the ground's normals being associated with an obstacle on top of it. BLEEDING edges.
        # Decreasing this (tried 0.0001) made the obstacle detection very unforgiving - to a fault as well. The "Under the hood" image was almost completely black.
        # A good value might be 0.05? 0.1?
        self.normFactor = 0.01
        
        # NOTE - not sure what this does. Something to do with surface NORMALS
        # Increasing this (tried 0.25) made the obstacle detection very UNforgiving
        # Decreasing this (tried 0.000025) Not sure what this did. No significant change.
        # A good value might be ? 0.0005? 0.001?
        self.curveFactor = 0.01
        
        # NOTE - not sure what this does. Something to do with surface NORMALS
        # ORIGINAL: 0.75
        # Increasing this (tried 7.5) NO object detection
        # Increasing this (tried 1) seemed to make object detection more forgiving. If so, not by much. The rotating trashcan was detected much easier.
        # Decreasing this (tried 0.075) didn't seem to do much... 
        # A good value might be 0.75? 
        self.dirPow = 0.75

        # NOTE - not sure what this does. Something to do with Obstacle detection
        # ORIGINAL: 0.5
        # Tried 5: Very dark "Under the hood" image. No object detection.
        # Tried 0.05: Not much change.
        self.combinePow = 0.5
        
        # NOTE - not sure what this does. Something to do with surface NORMALS and Obstacle Detection
        # ORIGINAL: (3, 3)
        # Tried (0.1, 0.1): MUST BE INT
        # Tried (1, 1): Put a lot MORE noise in the images
        # Tried (10, 10): Runtime error
        # Tried (5, 5): not much change, tbh. Maybe a little poorer obstacle detection?
        # Tried (1, 5): not too much change. Maybe a little better obstacle detection?
        # Tried (5, 1): ditto to (1, 5)
        # A good value might be (3, 3)
        self.gaussBlur = (3, 3)

        # NOTE - not sure what this does. Something to do with NORMALS and Obstacle Detection
        # ORIGINAL: 3
        # Tried 0.3: Must be INT and odd
        # Tried 1: Less "square" blur, or rather, less black squares in images. Not much change in object detection
        # Tried 31: MUCH MORE square blurs. The whole this was squares. However, it did increase detection in an unstable way.
        # Tried 11: more square blurs. A little better detection but quite unstable still.
        self.medBlur = 3

        # NOTE - not sure what this does. Something to do with NORMALS and Obstacle Detection
        # ORIGINAL: 320 .. maybe 3?
        # Tried 50: Not much change, tbh. The current detection capabilities seemed to be more constant - less flickering.
        # Tried 1000: Not much change, tbh. Maybe less detection?
        self.distToPixel = 320 / self.skip
        
        
        # NOTE - try removing the blurs alltogether. Using default values for everything else.
        # Original: both guassian and median blur
        # Tried only median blur: the median blur's squares are far more apparent now. Why are they so black? Do they need to be clamped? Better detection when rotating trashcan.
        # Tried only guassian blur: Some blurring (still some dark squares - clamping needed?). Poorer detection.
        # Neither: The dark squares are more apparent albeit smaller. Maybe the squares aren't due to the blurs? Detection seems more jittery but better!


        # NOTE - try removing the blurs alltogether. Using default values for everything else.
        # Original: both guassian and median blur
        # Tried only median blur: the median blur's squares are far more apparent now. Why are they so black? Do they need to be clamped? Better detection when rotating trashcan.
        # Tried only guassian blur: Some blurring (still some dark squares - clamping needed?). Poorer detection.
        # Neither: The dark squares are more apparent albeit smaller. Maybe the squares aren't due to the blurs? Detection seems more jittery but better!

        # Initialize blob detector
        self.initialize_blob_detect()

        # Subscriptions
        self.create_subscription(
            Image,
            f'/{self.zed_type}/zed_node/left/image_rect_color',
            self.imageCallback,
            10
        )
        self.create_subscription(
            Image,
            f'/{self.zed_type}/zed_node/depth/depth_registered',
            self.depthCallback,
            10
        )

        # Publishers
        self.pub_obstacles = self.create_publisher(ObstacleInfo, '/obstacles', 1)
        self.pub_zed_obs = self.create_publisher(ZedObstacles, '/zed/obstacles', 1)
        self.pub_obs_img_labeled = self.create_publisher(Image, '/zed_obstacle_image_labeled', 1)
        self.pub_obs_img_dir = self.create_publisher(Image, '/zed_obstacle_image_dir', 1)
        self.pub_obs_img_raw = self.create_publisher(Image, '/zed_obstacle_raw', 1)
        self.execute_timer = self.create_timer(self.execute_period, self.execute)

    def imageCallback(self, msg):
        try:
            self.get_logger().info("received imageCallback", once=True)
            self.img = self.bridge.imgmsg_to_cv2(msg, "passthrough")
            self.msgread["image"] = True
        except CvBridgeError as e:
            self.get_logger().info(str(e))

    def depthCallback(self, msg):
        try:
            self.get_logger().info("received depthCallback", once=True)
            self.depth_img = np.copy(self.bridge.imgmsg_to_cv2(msg, "passthrough"))
            self.msgread["depth"] = True

            # Remove ground from depth image, simplifying detection
            self.remove_ground(self.depth_img)
            # I put this here because anywhere else and the non-cut image would sometime get through
            # to the image processing. Not sure why, perhaps the thread is being paused during
            # image processing to recieve a new image and image processing is being
            # resumed on a new image? That would be pretty bad and should be fixed if it is the case.
            # I'm ignoring it for now only because the competition is in two days.
        except CvBridgeError as e:
            self.get_logger().info(str(e))

    ######## Helper Functions ########
    def display_img(self, name, img, scale=2):
        if self.display:
            img_scaled = img.repeat(scale,axis=0).repeat(scale,axis=1)
            cv2.imshow(name, img_scaled)
            cv2.waitKey(1)

    def initialize_blob_detect(self):
        #######
        # OpenCV's blob detection
        #######
        self.detectors = []
        # Set up the SimpleBlobdetector with default parameters.
        for j in range(self.distFrames):
            params = cv2.SimpleBlobDetector_Params()

            # Change thresholds
            params.minThreshold = 100
            params.maxThreshold = 256
            params.blobColor = 256

            # Filter by Area.
            params.filterByArea = True
            # NOTE: Original "6000 / self.skip
            params.minArea = (300 / self.skip) / (j + 1) # NOTE: Maybe if I decrease this number it would catch smaller objects?
            params.maxArea = (200000 / self.skip) / (j + 1) # NOTE: Maybe don't touch this one? 

            # Filter by Circularity
            params.filterByCircularity = False

            # Filter by Convexity
            params.filterByConvexity = False

            # Filter by Inertia
            params.filterByInertia = False

            #self.detector = cv2.SimpleBlobDetector_create(params)
            self.detectors.append(cv2.SimpleBlobDetector_create(params))

    def check_ready(self):
        ready = True
        for key in self.msgread:
            if not self.msgread[key]:
                ready = False
        self.ready = ready

    def filter_data(self):
        valid_pts = self.points[self.points[:, 0] != np.inf]
        if len(valid_pts) > 0:
            consec_points = self.consecutive(valid_pts[:, 1])
            obs_size = np.array([len(i) for i in consec_points])
            inds = np.where(obs_size > self.BALL_SIZE)[0]
            if len(inds) > 0:
                obs_angles = np.concatenate(consec_points[inds])
                self.obs = self.points[np.isin(self.points[:, 1], obs_angles)]
                self.xdists = self.obs[:, 0] * np.sin(np.deg2rad(self.obs[:, 1]))
        else:
            self.obs = np.zeros((0, 0))

    def determine_stuck(self):
        self.locations[:-1] = self.locations[1:]
        self.locations[-1] = [self.north, self.east]
        center = [np.median(self.locations[:, 0]), np.median(self.locations[:, 1])]

        rel_locations = self.locations - center
        if self.v != 0 and np.all(  # NOTE Is this ever true? self.v only ever = 0
            np.sqrt(rel_locations[:, 0] ** 2 + rel_locations[:, 1] ** 2)
            < self.STUCK_RADIUS
        ):
            self.is_stuck = True
        else:
            if self.is_stuck:
                self.locations[:, 0] = np.arange(self.LOCATION_ARRAY_SIZE)
            self.is_stuck = False

    def determine_wall(self):
        is_wall = len(self.obs) > 0
        self.prev_scores[0] = self.low_pass_filter(
            self.prev_scores[0], is_wall, self.alpha
        )
        self.is_wall = self.prev_scores[0] > 0.5

    def check_center(self):
        if len(self.obs) > 0:
            center_obs = np.any(abs(self.xdists) < self.ROVER_CLEARANCE)  # True
        else:
            center_obs = False

        self.prev_scores[1] = self.low_pass_filter(
            self.prev_scores[1], center_obs, self.alpha
        )
        self.center_obs = self.prev_scores[1] > 0.5

    def check_goal(self):
        # maybe limit to +/- 90 degree checks?
        if len(self.obs) > 0:
            goal_points = np.copy(self.obs)
            goal_points[:, 1] += int(self.heading_offset[1])

            if int(self.heading_offset[1]) >= 0:
                goal_points = goal_points[(0 + int(self.heading_offset[1])) :]
            elif int(self.heading_offset[1]) < 0:
                goal_points = goal_points[: (181 + int(self.heading_offset[1]))]

            goal_xdists = abs(goal_points[:, 0] * np.sin(np.deg2rad(goal_points[:, 1])))
            goal_obs = np.any(goal_xdists < self.ROVER_CLEARANCE)
        else:
            goal_obs = False

        self.prev_scores[2] = self.low_pass_filter(
            self.prev_scores[2], goal_obs, self.alpha
        )
        self.goal_obs = self.prev_scores[2] > 0.5

    def calc_obstacle_centroid(self):
        if len(self.obs) > 0:
            obstacle_centroid = int(np.average(self.obs[:, 1]))
        else:
            obstacle_centroid = 0

        self.prev_scores[3] = self.low_pass_filter(
            self.prev_scores[3], obstacle_centroid, self.alpha
        )
        self.obstacle_centroid = int(self.prev_scores[3])

    def calc_obstacle_weights(self):
        alpha = 0.5
        if len(self.obs) > 0:
            w_r = np.exp(-(((self.obs[:, 0] - self.RANGE_LOW) / 4) ** 2))
            w_x = np.exp(-self.xdists**2) * np.sign(self.xdists)
            w_x[w_x == 0] = np.sign(-self.heading_offset[0])

            weights = w_r * w_x

            pos_weights = weights[weights > 0]
            neg_weights = weights[weights < 0]
            pos = np.max(pos_weights) if pos_weights.size > 0 else 0
            neg = np.min(neg_weights) if neg_weights.size > 0 else 0
            pos_filt = self.low_pass_filter(self.prev_scores[4], pos, alpha)
            neg_filt = self.low_pass_filter(self.prev_scores[5], neg, alpha)

            self.prev_scores[4] = pos_filt
            self.prev_scores[5] = neg_filt

            if abs(pos_filt) > abs(neg_filt):
                weight = pos_filt
            else:
                weight = neg_filt
        else:
            weight = 0.0
            self.prev_scores[4] = self.low_pass_filter(self.prev_scores[4], 0, alpha)
            self.prev_scores[5] = self.low_pass_filter(self.prev_scores[5], 0, alpha)

        self.weight = weight

    def low_pass_filter(self, prev_filtered_score, current_score, filter_constant):
        alpha = filter_constant
        filt_score = (1 - alpha) * prev_filtered_score + alpha * current_score
        return filt_score

    def average_weight(self, weights):  # NOTE never used!
        if len(weights) > 0:
            return np.average(weights)
        else:
            return 0

    # Checks for distances that are consecutive angles in points array
    def consecutive(self, angles, stepsize=-1):
        return np.array(np.split(angles, np.where(np.diff(angles) != stepsize)[0] + 1))

    def get_normals(self):
        Max = self.maxDist + 2

        # Get depth image, skipping pixels to reduce image size
        depth_np = self.depth_img
        finite_np = np.nan_to_num(depth_np)  # Map inf and nan to 0 (black)

        # This flatens distance image (before edges appeared farther than center since diagonal distance longer)
        i1, i2 = depth_np.shape
        scaling_mat = np.sqrt(
            1
            + np.transpose(
                np.ones((i2, 1))
                * np.power(
                    (np.arange(i1) - (i1 - 1) / 2.0)
                    / self.distToPixel
                    / self.skip
                    / 6.0,
                    2,
                )
            )
            + np.ones((i1, 1))
            * np.power(
                (np.arange(i2) - (i2 - 1) / 2.0) / self.distToPixel / self.skip / 6.0, 2
            )
        )
        depth_np = np.divide(depth_np, scaling_mat)
        temp_curve = finite_np == 0  # for 2nd derivative, store zero entries
        temp = np.less(finite_np, Max * np.ones(depth_np.shape))  # Find distances < Max
        depth_np = np.multiply(
            temp.astype(int), finite_np
        )  # Shift distances > Max to 0

        # Create grayscale map of distance
        temp = depth_np != 0  # Find non-zero values
        depth_np = np.subtract(
            (Max) * temp.astype(int), depth_np
        )  # Subtract non-zero entries from Max
        #   (since we want small distances to white
        #    or large gray scale values)
        depth_np = cv2.GaussianBlur(
            depth_np, self.gaussBlur, 0
        )  # Blur image to smooth for derivatives

        #######
        # Surface Normals
        #######

        # Finite difference for surface normals (4th order accurate first derivative)
        dx = np.multiply(
            np.add(
                -1 * depth_np[2 : -3 : self.skip, 4 : -1 : self.skip],
                np.add(
                    8 * depth_np[2 : -3 : self.skip, 3 : -2 : self.skip],
                    np.add(
                        depth_np[2 : -3 : self.skip, 0 : -5 : self.skip],
                        -8 * depth_np[2 : -3 : self.skip, 1 : -4 : self.skip],
                    ),
                ),
            )
            / self.normFactor
            / 6,
            depth_np[2 : -3 : self.skip, 2 : -3 : self.skip],
        )
        dy = np.multiply(
            np.add(
                -1 * depth_np[4 : -1 : self.skip, 2 : -3 : self.skip],
                np.add(
                    8 * depth_np[3 : -2 : self.skip, 2 : -3 : self.skip],
                    np.add(
                        depth_np[0 : -5 : self.skip, 2 : -3 : self.skip],
                        -8 * depth_np[1 : -4 : self.skip, 2 : -3 : self.skip],
                    ),
                ),
            )
            / self.normFactor
            / 6,
            depth_np[2 : -3 : self.skip, 2 : -3 : self.skip],
        )
        i1, i2 = dx.shape

        # Combine normal data in rgb style array with (x,y,z) components
        vec = np.append(dx.reshape(i1, i2, 1), dy.reshape(i1, i2, 1), axis=2)
        vec = np.append(vec, np.ones((i1, i2, 1)), axis=2)
        norm = np.linalg.norm(vec, axis=2)  # Calc norm
        norm_vec = np.divide(
            vec, np.matmul(norm.reshape(i1, i2, 1), np.ones((1, 3)))
        )  # Normalize vectors

        # Neglect x-norm component to get only slope data
        slope_vec = np.append(dy.reshape(i1, i2, 1), np.ones((i1, i2, 1)), axis=2)
        slope_norm = np.linalg.norm(slope_vec, axis=2)  # Normalize
        slope_norm_vec = np.divide(
            slope_vec, np.matmul(slope_norm.reshape(i1, i2, 1), np.ones((1, 2)))
        )

        self.norm_img = np.uint8(128 * norm_vec + 127 * np.ones(norm_vec.shape))
        self.display_img('norm_image', self.norm_img)

        temp = norm_vec[:, :, 2] > self.dirThreshold
        slope_temp = slope_norm_vec[:, :, 1] > self.dirThreshold

        # Modify surface normals if gimbal tilted 
        if self.gimbal_tilt != 0:
            norm_vec, slope_norm_vec = self.modify_norms(norm_vec, slope_norm_vec)

        # Shift lower values closer to 1 by function mapping
        dir_im = np.uint8(
            255
            * (np.power(np.multiply(norm_vec[:, :, 2], temp.astype(int)), self.dirPow))
        )  # Power shift
        slope_im = np.uint8(
            255
            * (
                np.power(
                    np.multiply(slope_norm_vec[:, :, 1], slope_temp.astype(int)),
                    self.dirPow,
                )
            )
        )  # Power shift

        # Convert to rgb format and store
        self.dir_img = cv2.cvtColor(dir_im, cv2.COLOR_GRAY2BGR)
        self.display_img('dir_img', self.dir_img)
        self.slope_img = cv2.cvtColor(slope_im, cv2.COLOR_GRAY2BGR)
        self.display_img('slope_img', self.slope_img)

        #######
        # Curvature (norm of second derivative vectors)
        #######

        finite_np = cv2.GaussianBlur(
            finite_np, self.gaussBlur, 0
        )  # Blur to smooth for derivatives

        # Finite difference for curvature vectors (4th order accurate second derivative)
        dx2 = np.multiply(
            np.add(
                -30 * finite_np[2 : -3 : self.skip, 2 : -3 : self.skip],
                np.add(
                    -1 * finite_np[2 : -3 : self.skip, 4 : -1 : self.skip],
                    np.add(
                        16 * finite_np[2 : -3 : self.skip, 3 : -2 : self.skip],
                        np.add(
                            -1 * finite_np[2 : -3 : self.skip, 0 : -5 : self.skip],
                            16 * finite_np[2 : -3 : self.skip, 1 : -4 : self.skip],
                        ),
                    ),
                ),
            )
            / self.curveFactor
            / 12,
            finite_np[2 : -3 : self.skip, 2 : -3 : self.skip],
        )
        dy2 = np.multiply(
            np.add(
                -30 * finite_np[2 : -3 : self.skip, 2 : -3 : self.skip],
                np.add(
                    -1 * finite_np[4 : -1 : self.skip, 2 : -3 : self.skip],
                    np.add(
                        16 * finite_np[3 : -2 : self.skip, 2 : -3 : self.skip],
                        np.add(
                            -1 * finite_np[0 : -5 : self.skip, 2 : -3 : self.skip],
                            16 * finite_np[1 : -4 : self.skip, 2 : -3 : self.skip],
                        ),
                    ),
                ),
            )
            / self.curveFactor
            / 12,
            finite_np[2 : -3 : self.skip, 2 : -3 : self.skip],
        )

        # Combine components into (x,y,z) array
        vec2 = np.append(dy2.reshape(i1, i2, 1), dx2.reshape(i1, i2, 1), axis=2)
        vec2 = np.append(vec2, np.zeros((i1, i2, 1)), axis=2)

        # Map magnitude of curvature to grayscale
        norm = np.linalg.norm(vec2, axis=2)  # Calc norm
        mag_curve = norm / 2.0  # Scale norm
        temp = mag_curve < 1  # Find values in valid range (0,1)
        mag_curve = np.add(
            np.multiply(temp.astype(int), mag_curve), (1 - temp.astype(int))
        )
        curve_img = np.uint8(
            255
            * (
                (
                    np.subtract(
                        1 - mag_curve,
                        temp_curve.astype(int)[2 : -3 : self.skip, 2 : -3 : self.skip],
                    )
                    * 2
                    / 5
                )
                + 0.6
            )
        )
        self.curve_img = cv2.cvtColor(
            curve_img, cv2.COLOR_GRAY2BGR
        )  # Convert to rgb format
        self.curve_img = cv2.medianBlur(
            self.curve_img, self.medBlur
        )  # Blur to remove noise
        self.display_img('curve_img', self.curve_img)

    def modify_norms(self, norm_vec, slope_vec):
        phi = self.gimbal_tilt
        temp = np.cos(phi) * norm_vec[:, :, 1] + np.sin(phi) * norm_vec[:, :, 2]
        norm_vec[:, :, 2] = (
            -np.sin(phi) * norm_vec[:, :, 1] + np.cos(phi) * norm_vec[:, :, 2]
        )
        norm_vec[:, :, 1] = temp
        temp = np.cos(phi) * slope_vec[:, :, 0] + np.sin(phi) * slope_vec[:, :, 1]
        slope_vec[:, :, 1] = (
            -np.sin(phi) * slope_vec[:, :, 0] + np.cos(phi) * slope_vec[:, :, 1]
        )
        slope_vec[:, :, 0] = temp
        return norm_vec, slope_vec

    def depth_to_gray(self, i):
        depth_np = self.depth_img[2 : -3 : self.skip, 2 : -3 : self.skip]

        ######
        # Convert depth_img to grayscale
        ######
        finite_np = np.nan_to_num(depth_np)  # Remove infinite entries

        # Define intervals for distance map (depending on distace min, max, and number of frames)
        stepDist = (self.maxDist - self.minDist) / self.distFrames
        low = self.minDist + stepDist * i
        high = self.minDist + stepDist * (1 + i)

        # Apply overlap threshold to remove blindspots
        if i != 0:
            low = low - self.overlap
        if high < self.maxDist + self.overlap:
            high = high + self.overlap
        else:
            high = self.maxDist

        # Send too low and too high values to 0
        temp = np.logical_and(
            np.greater(finite_np, low * np.ones(depth_np.shape)),
            np.less(finite_np, high * np.ones(depth_np.shape)),
        )  # Remove too low and too high values
        finite_np = np.multiply(temp.astype(int), finite_np)
        temp1 = finite_np != 0
        finite_np = np.subtract((high) * temp1.astype(int), finite_np)
        gry = np.uint8(
            finite_np * 100.0 / (high - low) + 155.0 * temp1
        )  # Convert to depth image
        #   (last part ensures obstacles
        #    near back of threshold show up)

        self.gry_depth = cv2.cvtColor(
            gry, cv2.COLOR_GRAY2BGR
        )  # Convert to rgb format and store

    def remove_ground(self, depth_img):
        # Find max distance allowed for each row of pixels given a horizontal cutoff height
        pixel_height = depth_img.shape[0]
        row_angle = np.arange(1, np.ceil(pixel_height / 2)) * self.CAMERA_V_FOV / (pixel_height - 1)
        row_max_dist = self.groundDistCutoff / np.sin(row_angle)

        # Find the max distance allowed for each row of pixels given a slanted cutoff height
        theta = row_angle
        phi = self.groundAngleCutoff
        row_max_dist -= (self.groundDistCutoff * np.tan(phi) /
                         (np.tan(theta) * np.cos(theta) * (np.tan(phi) + np.tan(theta))))

        # Remove ground by setting dist to inf if past max dist
        for i in range(1, int(np.ceil(pixel_height / 2))):
            depth_img[-i, :] = np.where(depth_img[-i, :] > row_max_dist[-i], np.inf, depth_img[-i, :])

    def ZED_obstacle_detect(self):
        # Get depth map
        finite_np = np.nan_to_num(
            self.depth_img[2 : -3 : self.skip, 2 : -3 : self.skip]
        )

        self.display_img("depth map (with ground removed)", finite_np / 10)

        # Get normal image
        self.get_normals()
        n_h, n_w, _ = self.dir_img.shape

        

        # Check slope in close region 1 to RANGE_HIGH m in front of rover
        #    for determining whether should switch to advanced avoidance 
        temp = np.logical_and(
            np.greater(finite_np, 1.0 * np.ones(finite_np.shape)),
            np.less(finite_np, self.RANGE_HIGH * np.ones(finite_np.shape)),
        )
        close_slope = np.multiply(temp.astype(int), self.slope_img[:, :, 1])
        temp = np.greater(90.0 / 255.0 * close_slope, 45.0 * np.ones(finite_np.shape))

        if (
            np.sum(temp[int(n_h / 5) : int(4 * n_h / 5), int(n_w / 3) : int(2 * n_w / 3)]) > 1000
        ):  # if 2000 pixels have slope greater than 45
            self.close_slope = 89  # Artifact of how it was setup before, just needs to be higher than threshold in navigation
        else:
            self.close_slope = 0

        #######
        # Prep image for blob detection then blob detect
        #######

        # For each depth map partition (distace range), get grayscale and combine with normal data
        for j in range(self.distFrames):
            # Get gray image and combine with normal and curvature images
            self.depth_to_gray(j)
            self.gry_depth = np.uint8(
                np.power(
                    np.multiply(
                        np.float32(self.gry_depth),
                        np.multiply(
                            np.float32(self.curve_img), np.float32(self.slope_img)
                        ),
                    )
                    / (255.0**3),
                    self.combinePow,
                )
                * 255.0
            )

            # Blur, erode, and dilate image
            kernel = np.ones((self.medBlur + 2, self.medBlur + 2), np.uint8)
            comb_img = cv2.GaussianBlur(self.gry_depth, self.gaussBlur, 0)
            #comb_img = cv2.dilate(comb_img, kernel, iterations=1)
            #comb_img = cv2.erode(comb_img, kernel, iterations=1)

            # Run blob detector
            self.display_img(f"dist frame {j}", comb_img, 3)
            
            keypoints = self.detectors[j].detect(comb_img)
            #keypoints = self.detector.detect(comb_img)

            k_count = -1
            keyLoc = np.zeros((len(keypoints), 3))
            for point in keypoints:
                k_count += 1
                keyLoc[k_count, 0] = point.pt[0] / self.distToPixel
                keyLoc[k_count, 1] = point.pt[1] / self.distToPixel
                keyLoc[k_count, 2] = point.size / self.distToPixel

            # Combine distince partitions into single rgb image (with each partition using different color)
            if self.distFrames >= 3:
                if j == 0:
                    self.composite_depth = self.gry_depth[:, :, 0].reshape(n_h, n_w, 1)
                    self.composite_img = comb_img[:, :, 0].reshape(n_h, n_w, 1)
                    comp_np_first = comb_img
                elif j == 1:
                    self.composite_depth = np.append(
                        self.composite_depth,
                        self.gry_depth[:, :, 1].reshape(n_h, n_w, 1),
                        axis=2,
                    )
                    self.composite_img = np.append(
                        self.composite_img,
                        comb_img[:, :, 1].reshape(n_h, n_w, 1),
                        axis=2,
                    )
                    comp_np_second = comb_img
                elif j == 2:
                    self.composite_depth = np.append(
                        self.composite_depth,
                        self.gry_depth[:, :, 2].reshape(n_h, n_w, 1),
                        axis=2,
                    )
                    self.composite_img = np.append(
                        self.composite_img,
                        comb_img[:, :, 2].reshape(n_h, n_w, 1),
                        axis=2,
                    )
            self.gry_depth_shape = self.gry_depth.shape

            ######
            # Calculate slope and distance of images
            ######
            keymeas = np.zeros((len(keypoints), 2))
            k_count = -1
            for k in keypoints:
                k_count += 1
                box_rad = k.size * 0.30
                x1 = int(np.floor(k.pt[0] - box_rad))
                if x1 < 0:
                    x1 = 0
                x2 = int(np.floor(k.pt[0] + box_rad))
                if x2 >= n_w - 1:
                    x2 = n_w - 1
                y1 = int(np.floor(k.pt[1] - box_rad))
                if y1 < 0:
                    y1 = 0
                y2 = int(np.floor(k.pt[1] + box_rad))
                if y2 >= n_h - 1:
                    y2 = n_h - 1

                # Calc Distance
                temp = finite_np != 0
                dist = np.sum(finite_np[y1:y2, x1:x2]) / np.sum(temp[y1:y2, x1:x2])
                # dist = np.median(finite_np[x1:x2,y1:y2])
                if np.isnan(dist):
                    dist = self.minDist + (
                        self.maxDist - self.minDist
                    ) / self.distFrames * (j + 0.5)

                # Calc Slope
                median = np.median(np.nan_to_num(self.slope_img[y1:y2, x1:x2, 0]))
                temp = np.nan_to_num(self.slope_img[y1:y2, x1:x2, 0]) > median
                slope = np.sum(
                    np.multiply(
                        np.float32(self.slope_img[y1:y2, x1:x2, 0]) / 255.0,
                        temp.astype(float),
                    )
                ) / np.sum(temp.astype(float))
                slope = slope * 90

                keymeas[k_count, :] = [dist, slope]

            ######
            # Combine obstacles into single obstacle if close
            ######
            if j == 0:
                self.keyLoc = keyLoc
                self.keyPoints = keypoints
                self.keyMeas = keymeas

            else:
                k_count = -1
                for k in keyLoc:
                    k_count += 1
                    sk_count = -1
                    flag = True
                    for sk in self.keyLoc:
                        sk_count += 1
                        cent_dist = np.sqrt(
                            (k[0] - sk[0]) ** 2
                            + (k[1] - sk[1]) ** 2
                            + ((keymeas[k_count, 0] - self.keyMeas[sk_count, 0])) ** 2
                        )
                        if cent_dist < 0.3 * (k[2] + sk[2]):
                            flag = False
                    if flag:
                        self.keyLoc = np.append(self.keyLoc, [k], axis=0)
                        self.keyPoints = np.append(
                            self.keyPoints, [keypoints[k_count]], axis=0
                        )
                        self.keyMeas = np.append(
                            self.keyMeas, [keymeas[k_count, :]], axis=0
                        )

        #######
        # Publish images if any subscribers
        #######

        # Publish obstacle_detection image
        if self.pub_obs_img_labeled.get_num_connections() > 0 or self.display:
            image_np = self.img[2 : -3 : self.skip, 2 : -3 : self.skip]
            image_np_res = cv2.drawKeypoints(
                image_np,
                self.keyPoints,
                np.array([]),
                (0, 0, 255),
                cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS,
            )

            resize_factor = self.skip
            # Resize image for display
            image_np_res = cv2.resize(
                image_np_res,
                None,
                fx=resize_factor,
                fy=resize_factor,
                interpolation=cv2.INTER_AREA,
            )

            ######
            # Classify images, draw and label images
            ######
            k_count = -1
            for k in self.keyPoints:
                k_count += 1

                types = ["Obstacle", "Hill"]
                # Classify by size
                if (k.size * self.keyMeas[k_count, 0] > 1600) and (
                    self.keyMeas[k_count, 1] < 60
                ):
                    obs_type = types[1]
                else:
                    obs_type = types[0]

                cv2.putText(
                    image_np_res,
                    obs_type,
                    (
                        int((k.pt[0] - 5) * resize_factor),
                        int((k.pt[1] - 10) * resize_factor),
                    ),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.2 * resize_factor,
                    (0, 0, 255),
                    1,
                )
                cv2.putText(
                    image_np_res,
                    "Dist={:1.3}m".format(self.keyMeas[k_count, 0]),
                    (
                        int((k.pt[0] - 5) * resize_factor),
                        int((k.pt[1]) * resize_factor),
                    ),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.2 * resize_factor,
                    (0, 0, 255),
                    1,
                )
                cv2.putText(
                    image_np_res,
                    "Slope={:1.3} deg".format(self.keyMeas[k_count, 1]),
                    (
                        int((k.pt[0] - 5) * resize_factor),
                        int((k.pt[1] + 10) * resize_factor),
                    ),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.2 * resize_factor,
                    (0, 0, 255),
                    1,
                )

            # Publish labeled and compressed obstacle detection image <- "Obstacle Detection" display
            self.pub_obs_img_labeled.publish(
                self.bridge.cv2_to_imgmsg(image_np_res, encoding="bgra8")
            )

            self.display_img("Obstacle Detection", image_np_res, 1)

        # Publish 'under the hood' obstacle detection image
        if self.pub_obs_img_raw.get_num_connections() > 0:
            resize_factor = self.skip / 1.2
            comp_img_res = cv2.resize(
                self.composite_img,
                None,
                fx=resize_factor,
                fy=resize_factor,
                interpolation=cv2.INTER_AREA,
            )
            
            self.pub_obs_img_raw.publish(  # <- "Obstacle Detection (Under the Hood)" display
                self.bridge.cv2_to_imgmsg(comp_img_res, encoding="bgr8")
            )

        # Publish 'Obstacle Detection (Dir)' image
        if self.pub_obs_img_dir.get_num_connections() > 0:
            resize_factor = self.skip / 1.2
            dir_img_disp = cv2.resize(
                self.dir_img,
                None,
                fx=resize_factor,
                fy=resize_factor,
                interpolation=cv2.INTER_AREA,
            )
            self.pub_obs_img_dir.publish(  # <- "Obstacle Detection (Dir)" display
                self.bridge.cv2_to_imgmsg(dir_img_disp, encoding="bgr8")
            )
        
        if cv2.waitKey(1) & 0xFF == ord("q"):
            cv2.destroyAllWindows()
            self.shutdown = True
            return

        if not self.init:
            self.get_logger().warn("ObstacleDetection init successful!")
            self.init = True

    def getZEDObstacles(self):
        zed_msg = ZedObstacles()
        zed_msg.x_coord = self.keyLoc[:, 0] * self.distToPixel
        zed_msg.y_coord = self.keyLoc[:, 1] * self.distToPixel
        zed_msg.diameter = self.keyLoc[:, 2] * self.distToPixel
        zed_msg.slope = self.keyMeas[:, 1]
        zed_msg.dist = self.keyMeas[:, 0]
        zed_msg.gry_depth_height = self.gry_depth_shape[0]
        zed_msg.gry_depth_width = self.gry_depth_shape[1]
        zed_msg.close_slope = self.close_slope
        zed_msg.dist_to_pixel = self.distToPixel

        return zed_msg

    ######## Main Execute Function & Publish ########
    def execute(self):
        if self.ready:
            self.ZED_obstacle_detect()
            self.filter_data()
            self.determine_stuck()
            self.determine_wall()
            self.check_center()
            self.check_goal()
            self.calc_obstacle_centroid()
            self.calc_obstacle_weights()

            self.publish()
        else:
            self.get_logger().info(
                "ObstacleDetection: Waiting for ZED image and depth messages...", once=True
            )
            self.check_ready()
            if self.ready:
                self.get_logger().info("ObstacleDetection: ZED messages received -- READY!")
            # Maybe need to add a sleep here


    def publish(self):
        # Publish Obstacles
        obi_msg = ObstacleInfo()
        obi_msg.is_stuck = self.is_stuck
        obi_msg.is_wall = self.is_wall
        obi_msg.center_obs = self.center_obs
        obi_msg.goal_obs = self.goal_obs
        obi_msg.obstacle_centroid = self.obstacle_centroid
        obi_msg.weight = self.weight
        obi_msg.pos_weight = self.prev_scores[4]
        obi_msg.neg_weight = self.prev_scores[5]

        self.pub_obstacles.publish(obi_msg)

        # Publish ZED Obstacles
        zed_msg = self.getZEDObstacles()
        self.pub_zed_obs.publish(zed_msg)


# ===================================================
# Main ==============================================
# ===================================================

def main(args=None):
    rclpy.init(args=args)
    obstacle_detect = ObstacleDetect()
    rclpy.spin(obstacle_detect)
    obstacle_detect.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
