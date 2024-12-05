import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from .pcl_helper import *
import numpy as np
from scipy.spatial.transform import Rotation as R
import open3d as o3d

class HazardDetector(Node):
    def __init__(self):
        super().__init__('hazard_detector')
        
        self.declare_parameter('height_threshold', 0.5)
        self.declare_parameter('slope_threshold', 15.0)
        self.declare_parameter('bounding_box_length', 3.0)  
        self.declare_parameter('bounding_box_width', 1.5)  
        self.declare_parameter('bounding_box_height', 1.0)  

        self.height_threshold = self.get_parameter('height_threshold').value
        self.slope_threshold = np.radians(self.get_parameter('slope_threshold').value)
        self.box_length = self.get_parameter('bounding_box_length').value
        self.box_width = self.get_parameter('bounding_box_width').value
        self.box_height = self.get_parameter('bounding_box_height').value
                
        
        self.subscriber = self.create_subscription(
            PointCloud2,
            '/slam/point_cloud',
            self.point_cloud_callback,
            10
        )
        self.publisher = self.create_publisher(String, '/hazard_zones', 10)
        self.get_logger().info('Hazard_detector initialized')


    def point_cloud_callback(self, msg):
        # Convert PointCloud2 to PCL format
        cloud = ros_to_pcl(msg)
        
        # Downsample for efficiency
        voxel_filter = cloud.make_voxel_grid_filter()
        voxel_filter.set_leaf_size(0.1, 0.1, 0.1)  # Example: 10 cm grid
        cloud_filtered = voxel_filter.filter()
        
        # Segment ground plane
        seg = cloud_filtered.make_segmenter()
        seg.set_model_type(pcl.SACMODEL_PLANE)
        seg.set_method_type(pcl.SAC_RANSAC)
        seg.set_distance_threshold(0.05)  # Example: 5 cm tolerance
        
        inliers, coefficients = seg.segment()
        ground = cloud_filtered.extract(inliers, negative=False)
        non_ground = cloud_filtered.extract(inliers, negative=True)
        
        # Detect obstacles by height
        high_points = self.detect_high_obstacles(non_ground)
        
        # Detect steep slopes
        steep_slopes = self.detect_steep_slopes(ground, coefficients)
        
        # Publish hazards
        hazard_message = self.generate_hazard_message(high_points, steep_slopes)
        self.publisher.publish(hazard_message)

    def detect_high_obstacles(self, non_ground_cloud):
        # Find maximum height in each cluster
        clusters = self.cluster_point_cloud(non_ground_cloud)
        high_points = []
        
        for cluster in clusters:
            z_max = max(cluster[:, 2])  # Extract Z (height) component
            if z_max > self.height_threshold:
                high_points.append(cluster)
        
        return high_points

    def detect_steep_slopes(self, ground_cloud, plane_coefficients):
        # Calculate plane normal from coefficients (Ax + By + Cz + D = 0)
        normal = np.array(plane_coefficients[:3])
        z_axis = np.array([0, 0, 1])  # Assume Z is vertical
        
        # Angle between ground plane and vertical axis
        angle = np.arccos(np.dot(normal, z_axis) / (np.linalg.norm(normal) * np.linalg.norm(z_axis)))
        if angle > self.slope_threshold:
            return True
        return False

    def cluster_point_cloud(self, cloud):
        # Convert PointCloud2 to a NumPy array
        points = pcl_to_array(cloud)  # converts to Nx3 numpy array

        # Create an Open3D PointCloud object
        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(points)

        # Perform DBSCAN clustering (density-based clustering)
        labels = np.array(pc.cluster_dbscan(eps=0.05, min_points=10, print_progress=True))

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
        message = f"Hazards detected: "
        if high_points:
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
