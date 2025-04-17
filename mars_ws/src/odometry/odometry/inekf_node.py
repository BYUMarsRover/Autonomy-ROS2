import rclpy
from rclpy.node import Node
import numpy as np
from scipy.spatial.transform import Rotation as Rotation
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
# from tf_transformations import euler_from_quaternion, quaternion_multiply, quaternion_inverse, quaternion_matrix, quaternion_from_euler
from ublox_read_2.msg import PositionVelocityTime
from threading import Lock
# Use dedicated callback group for timer
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor


from collections import deque
import queue # Thread-safe python queue
import threading


# from tf2_ros import TransformBroadcaster, Buffer, TransformListener
# import tf2_ros
# from geometry_msgs.msg import TransformStamped

from odometry.ekf_helper.helper_functions import *
from odometry.ekf_helper.rover_inekf import InEKF
from odometry.ekf_helper.rover_sys import RoverSystem

'''
To look at: move to the robot localization navsat transform?
'''

class EKFNode(Node):
    def __init__(self):
        super().__init__('inekf_node')
        self.get_logger().info('Initializing InEKF Node')
        print('Initializing InEKF Node')

        ######### TF stuff #############
        # Initialize the transform broadcaster
        # self.tf_broadcaster = TransformBroadcaster(self)
        # self.tf_buffer = Buffer()
        # self.tf_listener = TransformListener(self.tf_buffer, self)

        self.rover_system = RoverSystem()
        self.inekf = InEKF(self.rover_system)

        self.init_state = np.zeros(9)
        self.start_filtering = [False, False]   # Dont start until we have GPS and Yaw
        self.imu_time = None
        # store only the last 10 gps positions
        self.gps_heading_recall = 3
        self.gps_pos = deque(maxlen= self.gps_heading_recall + 3)

        self.lock = Lock()  # Add mutex
        # self.sensor_buffer = deque()
        self.sensor_buffer = queue.Queue(maxsize=100) # Magic number, set to be high so it's no infinite

        # self.gps_origin = None
        self.ref_lat = 0.0
        self.ref_lon = 0.0

        # self.prev_odom = None

        self.new_imu = False
        self.new_gps = False

        imu_sub_cb = MutuallyExclusiveCallbackGroup()
        # self.gps_sub_cb = ReentrantCallbackGroup()

        self.sub_imu = self.create_subscription(Imu, 'zed/zed_node/imu/data', self.imu_callback, 10) #, callback_group=imu_sub_cb)  # Creates a subscriber to IMU node
        self.ublox_subscription = self.create_subscription(PositionVelocityTime, '/rover/PosVelTime', self.gps_callback, 10) #, callback_group=imu_sub_cb)  # Creates a subscriber to ublox node
        # self.sub_odom = self.create_subscription(Odometry, 'zed/zed_node/odom', self.odom_callback, 10)
        # self.sub_gps_odom = self.create_subscription(Odometry, 'odometry/gps', self.gps_odom_callback, 10)
        self.pub_state = self.create_publisher(Odometry, 'ekf/odom', 10)
        self.pub_gps_xy = self.create_publisher(PoseWithCovarianceStamped, 'gps/xy', 10)
        self.pub_gps_filtered = self.create_publisher(NavSatFix, 'gps/filtered', 10)

        self.main_thread = threading.Thread(target=self.main_processing_loop)
        self.main_thread.daemon = True  # Allow the main program to exit even if this thread is running
        self.main_thread.start()
        self.get_logger().info('InEKF Node Initialized')


        # self.pub_odom_in_map = self.create_publisher(Odometry, 'odom/map', 10)


        # timer_group = MutuallyExclusiveCallbackGroup()
        # self.timer = self.create_timer(1.0, self.ekf_loop)  # rates (GPS- 10Hz, Odom 12Hz, MAG 20hz, IMU )
    
    # def ekf_loop(self):
    #     if (all(self.start_filtering) and (self.inekf.filter_started == False)):
    #         self.inekf.initialize_filter(self.init_state, self.imu_time)
    #         self.get_logger().info('Initializing filter')

    #     if self.new_imu:
    #         self.new_imu = False
    #         self.process_imu(self.newest_imu)
    #     if self.new_gps:
    #         self.new_gps = False
    #         self.process_gps(self.newest_gps)
    #         if self.inekf.filter_started:
    #             self.publish_state(self.newest_gps.header.stamp)

    def main_processing_loop(self):
        self.get_logger().info('Starting main processing loop')
        while rclpy.ok():
            try:
                msg = self.sensor_buffer.get(timeout=1.0)  # Wait for a message from the queue. Timeout is for handling shutdown
                if (all(self.start_filtering) and (self.inekf.filter_started == False)):
                    self.inekf.initialize_filter(self.init_state, self.imu_time)
                    self.get_logger().info('Initializing filter')
                
                if isinstance(msg, PositionVelocityTime):
                    # Process GPS message
                    self.process_gps(msg)
                    if self.inekf.filter_started:
                        self.publish_state(msg.header.stamp)
                elif isinstance(msg, Imu):
                    # Process IMU message
                    self.process_imu(msg)
                else:
                    self.get_logger().warn('Unknown message type received in processing loop')
            except queue.Empty:
                pass # No message received, continue to next iteration

        self.get_logger().info('Processing loop terminated')


    def process_gps(self, msg: PositionVelocityTime):
        # Get heading from message
        time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        lat, lon, alt = [val.item() for val in msg.lla]

        hor_acc = msg.h_acc
        vert_acc = msg.v_acc

        vel_ned = msg.vel_ned # to incorporate later, need to check frames, assume global
        vel_acc = msg.s_acc

        head_mot = msg.head_mot # deg heading of motion (2-D)
        head_veh = msg.head_veh # deg heading of vehicle (2-D) (IGNORE: always 0)
        head_acc = msg.head_acc # heading accuracy estimate both motion and vehicle

        if self.start_filtering[0] == False:
            # self.get_logger().info('Initializing GPS, settings lat lon')
            # Convert latitude and longitude to local Cartesian coordinates (x, y)
            self.ref_lat = lat # Using first lat/lon as reference
            self.ref_lon = lon
            self.get_logger().info(f'GPS reference lat: {self.ref_lat}, lon: {self.ref_lon}')
            # x, y = convert_gps_to_xyz(lat, lon, self.ref_lat, self.ref_lon)
            self.init_state[6:] = [0, 0, alt] # In this implementation, we are treating the initial position as the first GPS reading
            self.start_filtering[0] = True

            # TODO handle a case where the GPS and IMU do not start at the same time

        else:   
            # self.get_logger().info(f'calc gps with lat lon: {lat}, {lon}')
            x, y = convert_gps_to_xyz(lat, lon, self.ref_lat, self.ref_lon)
            pose_gps = PoseWithCovarianceStamped()
            pose_gps.header.frame_id = 'gps'
            pose_gps.header.stamp = msg.header.stamp
            pose_gps.pose.pose.position.x = x
            pose_gps.pose.pose.position.y = y
            self.pub_gps_xy.publish(pose_gps)
            # Need to check if altitude is in the same units as the rover system (height above ellipsoid)

            if self.inekf.filter_started:
                self.inekf.gps_callback(x, y, alt, hor_acc, vert_acc)
                # self.inekf.heading_callback(head_mot+90, head_acc) # coordinate frame may be wrong, needs to be in ENU

                # Convert velocity from NED (North-East-Down) to ENU (East-North-Up)
                # (world is in ENU frame)
                vel_enu = np.array([vel_ned[1], vel_ned[0], -vel_ned[2]])
    
                self.inekf.gps_velocity_callback(vel_enu, vel_acc)

                # x, y = 0, 0
                self.gps_pos.append([x, y, alt])

                if len(self.gps_pos) > (self.gps_heading_recall - 1):
                    # atan2 function should give values in ENU frame
                    heading_diff = math.atan2(self.gps_pos[-1][1] - self.gps_pos[-self.gps_heading_recall][1], self.gps_pos[-1][0] - self.gps_pos[-self.gps_heading_recall][0])
                    self.inekf.heading_callback(np.rad2deg(heading_diff), head_acc)


    def gps_callback(self, msg: PositionVelocityTime):
        # timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        # lat, lon, alt = [val.item() for val in msg.lla]
        # hor_acc = msg.h_acc
        # vert_acc = msg.v_acc
        # vel_ned = msg.vel_ned
        # vel_acc = msg.s_acc

        # head_mot = msg.head_mot
        # head_acc = msg.head_acc
        self.sensor_buffer.put(msg) # Add to the queue
        # self.get_logger().info('GPS data added to buffer')

        # self.newest_gps = msg
        # if self.new_gps:
        #     self.get_logger().warn('Skipping GPS data because of higher rate than ekf checking')
        # else:
        #     self.new_gps = True
        
        # self.process_gps(msg)
        

        # while self.sensor_buffer:   
        #     msg = self.sensor_buffer.popleft()
        #     self.process_imu(msg)

        # # if self.new_imu:
        # #     self.new_imu = False
        # #     self.process_imu(self.newest_imu)
        
        # if self.inekf.filter_started:
        #     self.publish_state(msg.header.stamp)

        
        
        
    def process_imu(self, msg: Imu):
        time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.imu_time = time

        orientation = msg.orientation
        rpy = Rotation.from_quat([orientation.x, orientation.y, orientation.z, orientation.w]).as_euler('xyz', degrees=False)
        # R_matrix = Rotation.from_quat([orientation.x, orientation.y, orientation.z, orientation.w]).as_matrix()
        
        angular_velocity = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
        linear_acceleration = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]

        if self.inekf.filter_started:
            # TODO: Pass in heading into the filter when we are getting the heading from the IMU

            self.inekf.imu_callback(time, angular_velocity, linear_acceleration)
            # Ignore these functions for now, couldn't get them working well, could be a simple frame issue
            # or need params tuned. (Params are currently hardcoded in rover_inekf.py)
            # inekf.full_orientation_callback(time, R_matrix)
            # inekf.partial_orientation_gravity_callback(linear_acceleration)


        if self.start_filtering[1] == False:
            self.init_state[2] = rpy[2]
            self.start_filtering[1] = True


    def imu_callback(self, msg: Imu):
        # self.newest_imu = msg
        # if self.new_imu:
        #     self.get_logger().warn('Skipping IMU data because of higher rate than ekf checking')
        # else:    
        #     self.new_imu = True
        self.sensor_buffer.put(msg) # Add to the queue
        # self.get_logger().info('IMU data added to buffer')

    
    def publish_state(self, stamp):
        # Publish the state and covariance
        self.get_logger().info('Publishing state')
        mu = self.inekf.mu
        msg = Odometry()
        msg.header.frame_id = "map"
        msg.child_frame_id = "base_link"
        msg.header.stamp = stamp

        msg.pose.pose.position.x = mu[0, 4]
        msg.pose.pose.position.y = mu[1, 4]
        msg.pose.pose.position.z = mu[2, 4]

        msg.twist.twist.linear.x = mu[0, 3]
        msg.twist.twist.linear.y = mu[1, 3]
        msg.twist.twist.linear.z = mu[2, 3]
        rot = mu[:3, :3]
        q = Rotation.from_matrix(rot).as_quat()
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]
        self.pub_state.publish(msg)
        

        publish_filtered_gps = True
        if publish_filtered_gps:
            lat, lon = convert_xyz_to_gps(mu[0, 4], mu[1, 4], self.ref_lat, self.ref_lon)
            gps_msg = NavSatFix()
            gps_msg.header.frame_id = 'gps_link'
            gps_msg.header.stamp = stamp
            gps_msg.latitude = lat
            gps_msg.longitude = lon
            gps_msg.altitude = mu[2, 4]


            self.pub_gps_filtered.publish(gps_msg)



def main(args=None):
    rclpy.init(args=args)
    node = EKFNode()
    rclpy.spin(node)
    # executor = MultiThreadedExecutor()
    # executor.add_node(node)
    # executor.spin() # or rclpy.spin(node) for the default single-threaded executor
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
