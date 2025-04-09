import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from tf_transformations import euler_from_quaternion, quaternion_multiply, quaternion_inverse, quaternion_matrix, quaternion_from_euler
from ublox_read_2.msg import PositionVelocityTime


from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import tf2_ros
from geometry_msgs.msg import TransformStamped

class EKFNode(Node):
    def __init__(self):
        super().__init__('ekf_node')
        self.get_logger().info('Initializing EKF Node')
        print('Initializing EKF Node')

        ######### TF stuff #############
        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.gps_origin = None
        self.gps_received = False
        self.imu_received = False
        self.odom_received = False
        self.prev_odom = None

        self.start_filter = False

        self.new_imu = False
        self.new_odom = False
        self.new_gps = False
        self.new_gps_yaw = False
        
        # State: [x, y, yaw]
        self.X = np.zeros((3, 1))
        self.P = np.eye(3) * 0.1  # Covariance matrix
        
        self.sub_imu = self.create_subscription(Imu, 'imu/data', self.imu_callback, 10)
        self.sub_odom = self.create_subscription(Odometry, 'zed/zed_node/odom', self.odom_callback, 10)
        self.sub_gps_odom = self.create_subscription(Odometry, 'odometry/gps', self.gps_odom_callback, 10)
        self.pub_state = self.create_publisher(Odometry, 'ekf/odom', 10)
        self.pub_gps_xy = self.create_publisher(PoseWithCovarianceStamped, 'gps/xy', 10)
        self.pub_odom_in_map = self.create_publisher(Odometry, 'odom/map', 10)

        self.ublox_subscription = self.create_subscription(PositionVelocityTime, '/rover/PosVelTime', self.gps_callback, 10)  # Creates a subscriber to ublox node

        self.map_trans = None

        
        self.declare_ekf_parameters()
        self.timer = self.create_timer(0.11, self.ekf_loop)  # 9 Hz loop  Just slower than GPS rate (GPS- 10Hz, Odom 12Hz, MAG 20hz)
    
    def declare_ekf_parameters(self):
        #TODO: MAKE THE PARAMETERS IN THE CONFIG FILE
        self.declare_parameter('gps_noise', 0.2)  # 0.2 m
        self.declare_parameter('gps_yaw_noise_scalar', 1.0)  # 0.2 m
        self.declare_parameter('imu_noise', 0.01)  # 0.1 rad
        self.declare_parameter('odom_noise', 0.5) # 2.0 m
    
    def ekf_loop(self):
        self.start_filter = self.gps_received and self.imu_received and self.odom_received

        if self.new_odom:
            if self.start_filter:
                odom_noise = self.get_parameter('odom_noise').value
                odom_yaw_noise = 0.1
                dx, dy, dyaw = self.get_delta_odom(self.latest_odom)
                u = self.odom_to_map_transorm(dx, dy, dyaw)

                if u is not None:  # the case where the transform is not valid
                    F = np.eye(3)
                    B = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
                    # Q = np.eye(3) * odom_noise
                    Q = np.array([[odom_noise, 0, 0], [0, odom_noise, 0], [0, 0, odom_yaw_noise]])
                    # TODO fix my letters
                    self.kalman_predict(F, B, u, Q)

                    self.new_odom = False
                  
                if self.new_gps_yaw:
                    gps_yaw_noise = self.get_parameter('gps_yaw_noise_scalar').value * self.heading_acc_rad
                
                    H = np.array([[0, 0, 1]])
                    R = np.eye(1) * gps_yaw_noise
                    # self.kalman_update(self.z_gps_yaw, H, R)

                    self.new_gps_yaw = False

                if self.new_imu:
                    imu_noise = self.get_parameter('imu_noise').value
                
                    H = np.array([[0, 0, 1]])
                    R = np.eye(1) * imu_noise
                    self.kalman_update(self.z_imu, H, R)

                    self.new_imu = False
                
                if self.new_gps:
                    gps_noise = self.get_parameter('gps_noise').value

                    H = np.array([[1, 0, 0], [0, 1, 0]])
                    R = np.eye(2) * gps_noise
                    self.kalman_update(self.z_gps, H, R)

                    self.new_gps = False


            self.publish_state(self.latest_odom.header.stamp)
  

    def gps_callback(self, msg: PositionVelocityTime):
        # Get heading from message
        if msg.head_acc < 50.0:

            heading_deg_enu = (90 - msg.head_mot)

            heading_rad_enu = np.rad2deg(heading_deg_enu)
            self.heading_acc_rad = np.deg2rad(msg.head_acc)

            gps_yaw = self.wrap_angle(heading_rad_enu)

            self.z_gps_yaw = np.array([[gps_yaw]])

            self.new_gps_yaw = True

    
    def gps_odom_callback(self, msg: Odometry):
        if not self.gps_received:
            self.gps_received = True
            self.get_logger().info('Received First GPS data')
        
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y


        # LOW PASS FILTER?? What is the rate?
        self.z_gps = np.array([[x], [y]])

        self.new_gps = True

                
    def imu_callback(self, msg: Imu):
        if not self.imu_received:
            self.imu_received = True
            self.get_logger().info('Received First IMU data')
            _, _, yaw = euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
            self.yaw_value = yaw
            return
        
        _, _, yaw = euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        yaw = self.wrap_angle(yaw)

        self.alpha_yaw = 0.9

        self.yaw_value = self.yaw_value * self.alpha_yaw + (1.0 - self.alpha_yaw) * yaw

        self.z_imu = np.array([[self.yaw_value]])

        self.new_imu = True


    def publish_odom_transforms(self, msg: Odometry):
        t = TransformStamped()

        # TODO look at time and apply the frames from the message
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        t.transform.rotation = msg.pose.pose.orientation

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)

    
    def get_delta_odom(self, msg: Odometry):
        _, _, yaw = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        if self.prev_odom is not None:
            dx = msg.pose.pose.position.x - self.prev_odom[0]
            dy = msg.pose.pose.position.y - self.prev_odom[1]
            dyaw = self.get_angle_diff(yaw,  self.prev_odom[2])
            self.prev_odom = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        
        else:
            self.prev_odom = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
            return 0.0, 0.0, 0.0

        return dx, dy, dyaw
    

    def publish_odom_map_frame(self, msg: Odometry):
        try:
            # Lookup transformation from map to odom
            map_to_odom = self.tf_buffer.lookup_transform('map', 'odom', msg.header.stamp)
        
            if self.map_trans is None:
                # Extract translation and rotation from map → odom
                self.map_trans = np.array([
                    map_to_odom.transform.translation.x,
                    map_to_odom.transform.translation.y
                ])

            map_quat = [
                map_to_odom.transform.rotation.x,
                map_to_odom.transform.rotation.y,
                map_to_odom.transform.rotation.z,
                map_to_odom.transform.rotation.w
            ]
            _, _, map_yaw = euler_from_quaternion(map_quat)

            # Apply map → odom rotation to odom → base_link position
            R = np.array([
                [np.cos(map_yaw), -np.sin(map_yaw)],
                [np.sin(map_yaw),  np.cos(map_yaw)]
            ])

            # Extract odometry (odom → base_link)
            base_trans = np.array([
                msg.pose.pose.position.x,
                msg.pose.pose.position.y
            ])
            # base_quat = [
            #     msg.pose.pose.orientation.x,
            #     msg.pose.pose.orientation.y,
            #     msg.pose.pose.orientation.z,
            #     msg.pose.pose.orientation.w
            # ]
            # _, _, base_yaw = euler_from_quaternion(base_quat)

            base_trans_in_map = (R @ base_trans) + self.map_trans

            odom_map = Odometry()
            # Compute new orientation
            # base_yaw_in_map = map_yaw + base_yaw
            # new_quat = quaternion_from_euler(0, 0, base_yaw_in_map)
            # odom_map.pose.pose.orientation.x = new_quat[0]
            # odom_map.pose.pose.orientation.y = new_quat[1]
            # odom_map.pose.pose.orientation.z = new_quat[2]
            # odom_map.pose.pose.orientation.w = new_quat[3]

            # Publish new odometry (map → base_link)
            odom_map.header.frame_id = 'map'
            odom_map.child_frame_id = 'base_link'
            odom_map.header.stamp = msg.header.stamp
            odom_map.pose.pose.position.x = base_trans_in_map[0]
            odom_map.pose.pose.position.y = base_trans_in_map[1]

            self.pub_odom_in_map.publish(odom_map)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'Transform error: {str(e)}', throttle_duration_sec=10.0)
    

    # TODO: IS THIS ODOM TO MAP OR MAP TO ODOM?
    def odom_to_map_transorm(self, dx, dy, dyaw):
        # Transform odom data to map frame
        try:
            map_to_odom = self.tf_buffer.lookup_transform('map', 'odom', self.latest_odom.header.stamp)

            map_quat = [
                map_to_odom.transform.rotation.x,
                map_to_odom.transform.rotation.y,
                map_to_odom.transform.rotation.z,
                map_to_odom.transform.rotation.w
            ]
            _, _, map_yaw = euler_from_quaternion(map_quat)

            # Apply map → odom rotation to odom → base_link position
            R = np.array([
                [np.cos(map_yaw), -np.sin(map_yaw)],
                [np.sin(map_yaw),  np.cos(map_yaw)]
            ])
            
            # Rotate only dx, dy
            delta_pos = np.array([[dx], [dy]])
            delta_pos_transformed = R @ delta_pos  # Apply rotation

            # dyaw remains unchanged
            dyaw_map = dyaw
            dx_map, dy_map = delta_pos_transformed.flatten()
            u = np.array([[dx_map], [dy_map], [dyaw_map]])
            return u

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'Transform error: {str(e)}', throttle_duration_sec=10.0)

    
    def odom_callback(self, msg: Odometry):
        if not self.odom_received:
            self.get_logger().info('Received Odometry data')
            self.odom_received = True

        self.latest_odom = msg
        self.new_odom = True

        self.publish_odom_transforms(msg)
        # self.publish_odom_map_frame(msg)


    def kalman_predict(self, F, B, u, Q):
        self.X = F @ self.X + B @ u
        self.X[2, 0] = self.wrap_angle(self.X[2, 0])  # Wrap yaw
        self.P = F @ self.P @ F.T + Q
    
    
    def kalman_update(self, z, H, R):
        y = z - H @ self.X
        if H.shape[0] == 1:  # If updating yaw
            # This needs to handle the case where the yaw is just under pi on one side and just over -pi on the other
            y[0, 0] = self.wrap_angle(y[0, 0])

        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.X = self.X + K @ y
        self.X[2, 0] = self.wrap_angle(self.X[2, 0])  # Wrap yaw
        self.P = (np.eye(3) - K @ H) @ self.P

    def wrap_angle(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi
    
    def get_angle_diff(self, angle1, angle2):
        # return the difference between two angles but always the smallest difference (between -pi and pi)
        diff = angle1 - angle2
        return (diff + np.pi) % (2 * np.pi) - np.pi
    

    def publish_map_transforms(self, msg: Odometry):
        try:
            # Step 1: Get the `map -> base_link` transform from the odometry message
            q_map_to_base = [
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
            ]
            map_to_base_translation = [
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z
            ]

            # Convert `map -> base_link` rotation into a rotation matrix
            map_to_base_rot_matrix = quaternion_matrix(q_map_to_base)[:3, :3]  # Extract 3x3 rotation matrix

            # Step 2: Get the `base_link -> odom` transform from TF buffer
            base_to_odom = self.tf_buffer.lookup_transform(
                'base_link', 'odom', msg.header.stamp
            )

            q_base_to_odom = [
                base_to_odom.transform.rotation.x,
                base_to_odom.transform.rotation.y,
                base_to_odom.transform.rotation.z,
                base_to_odom.transform.rotation.w
            ]
            base_to_odom_translation = [
                base_to_odom.transform.translation.x,
                base_to_odom.transform.translation.y,
                base_to_odom.transform.translation.z
            ]

            # Step 3: Convert `base_link -> odom` translation into the `map` frame
            transformed_translation = map_to_base_rot_matrix @ base_to_odom_translation

            map_to_odom_translation = [
                map_to_base_translation[0] + transformed_translation[0],
                map_to_base_translation[1] + transformed_translation[1],
                map_to_base_translation[2] + transformed_translation[2]  # Assuming full 3D transform
            ]

            # Step 4: Compute the `map -> odom` rotation
            map_to_odom_rotation = quaternion_multiply(q_map_to_base, q_base_to_odom)

            # Step 5: Publish the `map -> odom` transform
            map_to_odom = TransformStamped()
            map_to_odom.header.stamp = msg.header.stamp
            map_to_odom.header.frame_id = 'map'
            map_to_odom.child_frame_id = 'odom'

            map_to_odom.transform.translation.x = map_to_odom_translation[0]
            map_to_odom.transform.translation.y = map_to_odom_translation[1]
            map_to_odom.transform.translation.z = map_to_odom_translation[2]

            map_to_odom.transform.rotation.x = map_to_odom_rotation[0]
            map_to_odom.transform.rotation.y = map_to_odom_rotation[1]
            map_to_odom.transform.rotation.z = map_to_odom_rotation[2]
            map_to_odom.transform.rotation.w = map_to_odom_rotation[3]

            self.tf_broadcaster.sendTransform(map_to_odom)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'Transform error: {str(e)}', throttle_duration_sec=10.0)
    
    
    def publish_state(self, stamp):
        msg = Odometry()
        msg.header.frame_id = "map"
        msg.child_frame_id = "base_link"
        msg.header.stamp = stamp
        msg.pose.pose.position.x = self.X[0, 0]
        msg.pose.pose.position.y = self.X[1, 0]
        msg.pose.pose.orientation.z = np.sin(self.X[2, 0] / 2)
        msg.pose.pose.orientation.w = np.cos(self.X[2, 0] / 2)
        self.pub_state.publish(msg)

        self.publish_map_transforms(msg)
        

def main(args=None):
    rclpy.init(args=args)
    node = EKFNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
