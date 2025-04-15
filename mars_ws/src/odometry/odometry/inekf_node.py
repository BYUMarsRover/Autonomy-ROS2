import rclpy
from rclpy.node import Node
import numpy as np
from scipy.spatial.transform import Rotation as Rotation
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
# from tf_transformations import euler_from_quaternion, quaternion_multiply, quaternion_inverse, quaternion_matrix, quaternion_from_euler
from ublox_read_2.msg import PositionVelocityTime

from collections import deque

# from tf2_ros import TransformBroadcaster, Buffer, TransformListener
# import tf2_ros
# from geometry_msgs.msg import TransformStamped

from odometry.inekf.helper_functions import *
from odometry.inekf.rover_inekf import InEKF
from odometry.inekf.rover_sys import RoverSystem

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
        self.gps_heading_recall = 5
        self.gps_pos = deque(maxlen= self.gps_heading_recall + 5)

        self.sensor_buffer = deque()

        # self.gps_origin = None
        self.ref_lat = 0.0
        self.ref_lon = 0.0

        # self.prev_odom = None

        self.sub_imu = self.create_subscription(Imu, 'zed/zed_node/imu/data', self.imu_callback, 10)
        self.ublox_subscription = self.create_subscription(PositionVelocityTime, '/rover/PosVelTime', self.gps_callback, 10)  # Creates a subscriber to ublox node
        # self.sub_odom = self.create_subscription(Odometry, 'zed/zed_node/odom', self.odom_callback, 10)
        # self.sub_gps_odom = self.create_subscription(Odometry, 'odometry/gps', self.gps_odom_callback, 10)
        self.pub_state = self.create_publisher(Odometry, 'ekf/odom', 10)
        self.pub_gps_xy = self.create_publisher(PoseWithCovarianceStamped, 'gps/xy', 10)
        self.pub_odom_in_map = self.create_publisher(Odometry, 'odom/map', 10)


        # self.declare_ekf_parameters()
        self.timer = self.create_timer(0.05, self.ekf_loop)  # 9 Hz loop  Just slower than GPS rate (GPS- 10Hz, Odom 12Hz, MAG 20hz, IMU )
    
    def declare_ekf_parameters(self):
        #TODO: MAKE THE PARAMETERS IN THE CONFIG FILE
        self.declare_parameter('gps_noise', 0.2)  # 0.2 m
        self.declare_parameter('gps_yaw_noise_scalar', 1.0)  # 0.2 m
        self.declare_parameter('imu_noise', 0.01)  # 0.1 rad
        self.declare_parameter('odom_noise', 0.5) # 2.0 m
    
    
    def ekf_loop(self):
        
        if (all(self.start_filtering) and (self.inekf.filter_started == False)):
            self.inekf.initialize_filter(self.init_state, self.imu_time)
            self.get_logger().info('Initializing filter')

        while len(self.sensor_buffer) > 0:
            item = self.sensor_buffer.popleft()
            if isinstance(item, Imu):
                self.process_imu(item)
                # self.get_logger().info('Processing IMU data')
            elif isinstance(item, PositionVelocityTime):
                self.process_gps(item)
                # self.get_logger().info('Processing GPS data')
                if self.inekf.filter_started:
                    self.publish_state(item.header.stamp)


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
            # Convert latitude and longitude to local Cartesian coordinates (x, y)
            self.ref_lat = lat # Using first lat/lon as reference
            self.ref_lon = lon
            # x, y = convert_gps_to_xyz(lat, lon, self.ref_lat, self.ref_lon)
            self.init_state[6:] = [0, 0, alt] # In this implementation, we are treating the initial position as the first GPS reading
            self.start_filtering[0] = True

            # TODO handle a case where the GPS and IMU do not start at the same time

        else:   
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

                self.gps_pos.append([x, y, alt])

                if len(self.gps_pos) > (self.gps_heading_recall - 1):
                    # atan2 function should give values in ENU frame
                    heading_diff = math.atan2(self.gps_pos[-1][1] - self.gps_pos[-self.gps_heading_recall][1], self.gps_pos[-1][0] - self.gps_pos[-self.gps_heading_recall][0])
                    self.inekf.heading_callback(np.rad2deg(heading_diff), head_acc)


    def gps_callback(self, msg: PositionVelocityTime):

        self.sensor_buffer.append(msg)
        # self.get_logger().info('Received GPS data')



    # def gps_odom_callback(self, msg: Odometry):
    #     if not self.gps_received:
    #         self.gps_received = True
    #         self.get_logger().info('Received First GPS data')
        
    #     x = msg.pose.pose.position.x
    #     y = msg.pose.pose.position.y


    #     # LOW PASS FILTER?? What is the rate?
    #     self.z_gps = np.array([[x], [y]])

    #     self.new_gps = True

    def process_imu(self, msg: Imu):
        time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.imu_time = time

        orientation = msg.orientation
        rpy = Rotation.from_quat([orientation.x, orientation.y, orientation.z, orientation.w]).as_euler('xyz', degrees=False)
        R_matrix = Rotation.from_quat([orientation.x, orientation.y, orientation.z, orientation.w]).as_matrix()
        
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
        self.sensor_buffer.append(msg)
        # self.get_logger().info('Received IMU data')


    # def publish_odom_transforms(self, msg: Odometry):
    #     t = TransformStamped()

    #     # TODO look at time and apply the frames from the message
    #     t.header.stamp = msg.header.stamp
    #     t.header.frame_id = 'odom'
    #     t.child_frame_id = 'base_link'

    #     t.transform.translation.x = msg.pose.pose.position.x
    #     t.transform.translation.y = msg.pose.pose.position.y
    #     t.transform.translation.z = msg.pose.pose.position.z

    #     t.transform.rotation = msg.pose.pose.orientation

    #     # Send the transformation
    #     self.tf_broadcaster.sendTransform(t)

    
    # def get_delta_odom(self, msg: Odometry):
    #     _, _, yaw = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
    #     if self.prev_odom is not None:
    #         dx = msg.pose.pose.position.x - self.prev_odom[0]
    #         dy = msg.pose.pose.position.y - self.prev_odom[1]
    #         dyaw = self.get_angle_diff(yaw,  self.prev_odom[2])
    #         self.prev_odom = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        
    #     else:
    #         self.prev_odom = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
    #         return 0.0, 0.0, 0.0

    #     return dx, dy, dyaw
    

    # def publish_odom_map_frame(self, msg: Odometry):
    #     try:
    #         # Lookup transformation from map to odom
    #         map_to_odom = self.tf_buffer.lookup_transform('map', 'odom', msg.header.stamp)
        
    #         if self.map_trans is None:
    #             # Extract translation and rotation from map → odom
    #             self.map_trans = np.array([
    #                 map_to_odom.transform.translation.x,
    #                 map_to_odom.transform.translation.y
    #             ])

    #         map_quat = [
    #             map_to_odom.transform.rotation.x,
    #             map_to_odom.transform.rotation.y,
    #             map_to_odom.transform.rotation.z,
    #             map_to_odom.transform.rotation.w
    #         ]
    #         _, _, map_yaw = euler_from_quaternion(map_quat)

    #         # Apply map → odom rotation to odom → base_link position
    #         R = np.array([
    #             [np.cos(map_yaw), -np.sin(map_yaw)],
    #             [np.sin(map_yaw),  np.cos(map_yaw)]
    #         ])

    #         # Extract odometry (odom → base_link)
    #         base_trans = np.array([
    #             msg.pose.pose.position.x,
    #             msg.pose.pose.position.y
    #         ])
    #         # base_quat = [
    #         #     msg.pose.pose.orientation.x,
    #         #     msg.pose.pose.orientation.y,
    #         #     msg.pose.pose.orientation.z,
    #         #     msg.pose.pose.orientation.w
    #         # ]
    #         # _, _, base_yaw = euler_from_quaternion(base_quat)

    #         base_trans_in_map = (R @ base_trans) + self.map_trans

    #         odom_map = Odometry()
    #         # Compute new orientation
    #         # base_yaw_in_map = map_yaw + base_yaw
    #         # new_quat = quaternion_from_euler(0, 0, base_yaw_in_map)
    #         # odom_map.pose.pose.orientation.x = new_quat[0]
    #         # odom_map.pose.pose.orientation.y = new_quat[1]
    #         # odom_map.pose.pose.orientation.z = new_quat[2]
    #         # odom_map.pose.pose.orientation.w = new_quat[3]

    #         # Publish new odometry (map → base_link)
    #         odom_map.header.frame_id = 'map'
    #         odom_map.child_frame_id = 'base_link'
    #         odom_map.header.stamp = msg.header.stamp
    #         odom_map.pose.pose.position.x = base_trans_in_map[0]
    #         odom_map.pose.pose.position.y = base_trans_in_map[1]

    #         self.pub_odom_in_map.publish(odom_map)

    #     except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
    #         self.get_logger().warn(f'Transform error: {str(e)}', throttle_duration_sec=10.0)
    

    # # TODO: IS THIS ODOM TO MAP OR MAP TO ODOM?
    # def odom_to_map_transorm(self, dx, dy, dyaw):
    #     # Transform odom data to map frame
    #     try:
    #         map_to_odom = self.tf_buffer.lookup_transform('map', 'odom', self.latest_odom.header.stamp)

    #         map_quat = [
    #             map_to_odom.transform.rotation.x,
    #             map_to_odom.transform.rotation.y,
    #             map_to_odom.transform.rotation.z,
    #             map_to_odom.transform.rotation.w
    #         ]
    #         _, _, map_yaw = euler_from_quaternion(map_quat)

    #         # Apply map → odom rotation to odom → base_link position
    #         R = np.array([
    #             [np.cos(map_yaw), -np.sin(map_yaw)],
    #             [np.sin(map_yaw),  np.cos(map_yaw)]
    #         ])
            
    #         # Rotate only dx, dy
    #         delta_pos = np.array([[dx], [dy]])
    #         delta_pos_transformed = R @ delta_pos  # Apply rotation

    #         # dyaw remains unchanged
    #         dyaw_map = dyaw
    #         dx_map, dy_map = delta_pos_transformed.flatten()
    #         u = np.array([[dx_map], [dy_map], [dyaw_map]])
    #         return u

    #     except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
    #         self.get_logger().warn(f'Transform error: {str(e)}', throttle_duration_sec=10.0)

    
    # def odom_callback(self, msg: Odometry):
    #     if not self.odom_received:
    #         self.get_logger().info('Received Odometry data')
    #         self.odom_received = True

    #     self.latest_odom = msg
    #     self.new_odom = True

    #     self.publish_odom_transforms(msg)
    #     # self.publish_odom_map_frame(msg)

    # def publish_map_transforms(self, msg: Odometry):
    #     try:
    #         # Step 1: Get the `map -> base_link` transform from the odometry message
    #         q_map_to_base = [
    #             msg.pose.pose.orientation.x,
    #             msg.pose.pose.orientation.y,
    #             msg.pose.pose.orientation.z,
    #             msg.pose.pose.orientation.w
    #         ]
    #         map_to_base_translation = [
    #             msg.pose.pose.position.x,
    #             msg.pose.pose.position.y,
    #             msg.pose.pose.position.z
    #         ]

    #         # Convert `map -> base_link` rotation into a rotation matrix
    #         map_to_base_rot_matrix = quaternion_matrix(q_map_to_base)[:3, :3]  # Extract 3x3 rotation matrix

    #         # Step 2: Get the `base_link -> odom` transform from TF buffer
    #         base_to_odom = self.tf_buffer.lookup_transform(
    #             'base_link', 'odom', msg.header.stamp
    #         )

    #         q_base_to_odom = [
    #             base_to_odom.transform.rotation.x,
    #             base_to_odom.transform.rotation.y,
    #             base_to_odom.transform.rotation.z,
    #             base_to_odom.transform.rotation.w
    #         ]
    #         base_to_odom_translation = [
    #             base_to_odom.transform.translation.x,
    #             base_to_odom.transform.translation.y,
    #             base_to_odom.transform.translation.z
    #         ]

    #         # Step 3: Convert `base_link -> odom` translation into the `map` frame
    #         transformed_translation = map_to_base_rot_matrix @ base_to_odom_translation

    #         map_to_odom_translation = [
    #             map_to_base_translation[0] + transformed_translation[0],
    #             map_to_base_translation[1] + transformed_translation[1],
    #             map_to_base_translation[2] + transformed_translation[2]  # Assuming full 3D transform
    #         ]

    #         # Step 4: Compute the `map -> odom` rotation
    #         map_to_odom_rotation = quaternion_multiply(q_map_to_base, q_base_to_odom)

    #         # Step 5: Publish the `map -> odom` transform
    #         map_to_odom = TransformStamped()
    #         map_to_odom.header.stamp = msg.header.stamp
    #         map_to_odom.header.frame_id = 'map'
    #         map_to_odom.child_frame_id = 'odom'

    #         map_to_odom.transform.translation.x = map_to_odom_translation[0]
    #         map_to_odom.transform.translation.y = map_to_odom_translation[1]
    #         map_to_odom.transform.translation.z = map_to_odom_translation[2]

    #         map_to_odom.transform.rotation.x = map_to_odom_rotation[0]
    #         map_to_odom.transform.rotation.y = map_to_odom_rotation[1]
    #         map_to_odom.transform.rotation.z = map_to_odom_rotation[2]
    #         map_to_odom.transform.rotation.w = map_to_odom_rotation[3]

    #         self.tf_broadcaster.sendTransform(map_to_odom)

    #     except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
    #         self.get_logger().warn(f'Transform error: {str(e)}', throttle_duration_sec=10.0)
    
    
    def publish_state(self, stamp):
        # Publish the state and covariance
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

        # self.publish_map_transforms(msg)
        

def main(args=None):
    rclpy.init(args=args)
    node = EKFNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
