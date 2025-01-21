import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Imu
from std_msgs.msg import String
from .pcl_helper import *
import numpy as np
from scipy.spatial.transform import Rotation as R
import open3d as o3d
from rover_msgs.msg import Hazard, HazardArray

class HazardDetector(Node):
    def __init__(self):
        super().__init__('hazard_detector')
        
        #Parameters
        self.declare_parameter('height_threshold', 0.5) #Height of hazard to detect
        self.declare_parameter('slope_threshold', 15.0) #Height of max slope of ground
        self.declare_parameter('voxel_grid_size', .1)
        self.declare_parameter('ground_distance_threshold', .05) #Increase if the ground is pretty rocky
        self.declare_parameter('bounding_box_start_point', [-0.5, 0.0, 0.5]) #X, Y, Z- X is up the mast, Y is to the right from the rovers perspective, and Z is out away from the rover, all in the Lidars' orientation
        self.declare_parameter('bounding_box_length', 3.0)  
        self.declare_parameter('bounding_box_width', 1.5)  
        self.declare_parameter('bounding_box_height', 1.0)
        self.declare_parameter('epsilon_clustering_density', 1.0)  #Change if the density of the points is not large enough to detect hazards
        self.declare_parameter('min_points_to_cluster', 10) #Increase if we are getting false positives for hazard detection, decrease if we aren't detecting anything
        

        self.height_threshold = self.get_parameter('height_threshold').value
        self.slope_threshold = np.radians(self.get_parameter('slope_threshold').value)
        self.voxel_grid_size = self.get_parameter('voxel_grid_size').value
        self.ground_distance_threshold = self.get_parameter('ground_distance_threshold').value
        self.bounding_box_start_point = self.get_parameter('bounding_box_start_point').value
        self.box_length = self.get_parameter('bounding_box_length').value
        self.box_width = self.get_parameter('bounding_box_width').value
        self.box_height = self.get_parameter('bounding_box_height').value
        self.epsilon_clustering_density = self.get_parameter('epsilon_clustering_density').value
        self.min_points_to_cluster = self.get_parameter('min_points_to_cluster').value

        #Initialize other variables
        self.orientation = None   

        #Subscribers
        self.subscriber = self.create_subscription(
            PointCloud2,
            '/unilidar/cloud',
            self.point_cloud_callback,
            10
        )
        self.imu_subscriber = self.create_subscription(
            Imu,
            '/unilidar/imu',
            self.imu_callback,
            10
        )

        #Publishers
        self.publisher = self.create_publisher(HazardArray, '/hazards', 10)

        self.get_logger().info('Hazard_detector initialized')


    def imu_callback(self, msg):
        self.orientation = msg.orientation

    def point_cloud_callback(self, msg):
        # Convert PointCloud2 to Open3D format and transform to the bounding box frame
        cloud = ros_to_pcl_and_transform(msg, self.bounding_box_start_point)

        # Downsample for efficiency
        cloud_filtered = cloud.voxel_down_sample(voxel_size=self.voxel_grid_size)

        # Convert point cloud to numpy array
        transformed_points = np.asarray(cloud_filtered.points)

        #Creates an array of booleans of whether the point at that index is in the bounding box or not
        bounding_mask = (
            (0 <= transformed_points[:, 2]) & (transformed_points[:, 2] <= self.box_length) &  # Z-axis (length)
            (-self.box_width / 2 <= transformed_points[:, 1]) & (transformed_points[:, 1] <= self.box_width / 2) &  # Y-axis (width)
            (transformed_points[:, 0] <= self.box_height)  # X-axis (height)
        )

        #Get cloud of only points in the bounding box
        cloud_bounded = cloud_filtered.select_by_index(np.where(bounding_mask)[0].tolist())

        o3d.visualization.draw_geometries([cloud_bounded])

        # Segment ground plane
        plane_model, inliers = cloud_bounded.segment_plane(
            distance_threshold=self.ground_distance_threshold,
            ransac_n=3,
            num_iterations=1000
        )
        ground = cloud_bounded.select_by_index(inliers)
        non_ground = cloud_bounded.select_by_index(inliers, invert=True)

        # Detect obstacles by height
        high_points = self.detect_high_obstacles(non_ground)

        # Detect steep slopes
        steep_slopes = self.detect_steep_slopes(ground, plane_model)

        # Publish hazards
        hazard_message = self.generate_hazard_message(high_points, steep_slopes)
        self.publisher.publish(hazard_message)

    def detect_high_obstacles(self, non_ground_cloud):
        # Find maximum height in each cluster

        clusters = self.cluster_point_cloud(non_ground_cloud)
        high_points = []

        self.get_logger().info(f"Detected {len(clusters)} clusters (potential hazards).")
        
        for cluster in clusters:
            z_max = max(cluster[:, 2])  # Extract Z (height) component
            if z_max > self.height_threshold:
                high_points.append(cluster)

        self.get_logger().info(f"Detected {len(high_points)} hazards that are above height {self.height_threshold} meters.")
        
        return high_points

    def detect_steep_slopes(self, ground_cloud, plane_coefficients):
        # Calculate plane normal from coefficients (Ax + By + Cz + D = 0)
        normal = np.array(plane_coefficients[:3])
        z_axis = np.array([0, 0, 1])  # Assume Z is vertical
        
        # Angle between ground plane and vertical axis
        angle = np.arccos(np.dot(normal, z_axis) / (np.linalg.norm(normal) * np.linalg.norm(z_axis)))

        self.get_logger().info(f"Angle of the ground is: {angle} radians, and the slope threshold is {np.radians(self.slope_threshold)}")
        if angle > np.radians(self.slope_threshold):
            return True
        return False

    def cluster_point_cloud(self, cloud):
        # Convert PointCloud2 to a NumPy array
        points = pcl_to_array(cloud)  # converts to Nx3 numpy array

        self.get_logger().info(f"Point cloud contains {points.shape[0]} points.")

        if points.shape[0] == 0:
            self.get_logger().warn("Point cloud is empty after conversion!")
            return []

        # Create an Open3D PointCloud object
        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(points)

        # Perform DBSCAN clustering (density-based clustering)
        labels = np.array(pc.cluster_dbscan(eps=self.epsilon_clustering_density, min_points=self.min_points_to_cluster, print_progress=True))

        # Extract clusters
        clusters = []
        unique_labels = np.unique(labels)
        for label in unique_labels:
            if label == -1:  # Ignore noise
                continue
            cluster_points = points[labels == label]
            clusters.append(cluster_points)

        return clusters

    def generate_hazard_message(self, high_points, steep_slopes):
        #TODO Work on this next to have hazards published according to the custom msgs that we made
        message = HazardArray()
        

        #message = f"Hazards detected: "
        if high_points:
            for high_point in high_points:
                # Create a hazard message for each high point by averaging the x, y, z coordinates
                hazard = Hazard()
                hazard.type = Hazard.OBSTACLE
                hazard.location_x = np.mean(high_point[:, 0])
                hazard.location_y = np.mean(high_point[:, 1])
                hazard.location_z = np.mean(high_point[:, 2])
                hazard.radius = np.max(np.std(high_point, axis=0))
                message.hazards.append(hazard)
            message += f"{len(high_points)} high obstacles. "
        if steep_slopes:
            message += "Steep slopes detected."
        return String(data=message)

def main(args=None):
    rclpy.init(args=args)
    node = HazardDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
