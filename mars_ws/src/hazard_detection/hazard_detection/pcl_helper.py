import numpy as np
import pcl
from sensor_msgs.msg import PointCloud2
import struct
import open3d as o3d


def ros_to_pcl(ros_cloud):
    """
    Converts a ROS PointCloud2 message to an Open3D PointCloud.
    
    :param ros_cloud: ROS PointCloud2 message
    :return: Open3D PointCloud
    """
    points_list = []

    # Extract points from ROS PointCloud2 message
    for point in read_points(ros_cloud, skip_nans=True):
        points_list.append([point[0], point[1], point[2]])  # x, y, z

    # Convert to NumPy array
    np_points = np.array(points_list, dtype=np.float32)

    # Create Open3D PointCloud
    pcl_data = o3d.geometry.PointCloud()
    pcl_data.points = o3d.utility.Vector3dVector(np_points)
    
    # Generate the fake point cloud
    fake_pc = generate_fake_point_cloud()

    # Visualize the point cloud
    visualize_point_cloud(fake_pc)


    #return pcl_data
    return fake_pc #Update when actually using a point cloud

def visualize_point_cloud(pc):
    # Visualize the point cloud
    o3d.visualization.draw_geometries([pc])

def generate_fake_point_cloud():
    # Create ground plane points (z = 0)
    ground_plane = np.random.uniform(-5, 5, (1000, 2))  # 1000 points on the ground plane
    ground_plane = np.hstack((ground_plane, np.zeros((ground_plane.shape[0], 1))))

    # Create non-ground points (random z values)
    non_ground = np.random.uniform(-5, -4, (200, 2))  # 200 non-ground points
    heights = np.random.uniform(1, 5, (non_ground.shape[0], 1))  # z values for non-ground
    non_ground = np.hstack((non_ground, heights))

    #Extra non_ground
    non_ground2 = np.random.uniform(4, 5, (200, 2))  # 200 non-ground points
    heights2 = np.random.uniform(1, 3, (non_ground.shape[0], 1))  # z values for non-ground
    non_ground2 = np.hstack((non_ground2, heights2))

    # Combine ground and non-ground points
    all_points = np.vstack((ground_plane, non_ground, non_ground2))

    # Convert to Open3D PointCloud
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(all_points)

    return pc

def pcl_to_ros(pcl_cloud):
    """
    Converts a PCL PointCloud to a ROS PointCloud2 message.
    
    :param pcl_cloud: PCL PointCloud
    :return: ROS PointCloud2 message
    """
    points = np.asarray(pcl_cloud)
    ros_cloud = create_cloud_xyz32(points)
    return ros_cloud

def read_points(cloud, skip_nans=True):
    """
    Generator to iterate through PointCloud2 data.

    :param cloud: PointCloud2 message
    :param skip_nans: Skip NaN values
    :yield: Point as a tuple (x, y, z)
    """
    fmt = "<fff"  # XYZ format
    point_step = cloud.point_step
    row_step = cloud.row_step
    data = cloud.data
    for row in range(cloud.height):
        for col in range(cloud.width):
            i = row * row_step + col * point_step
            x, y, z = struct.unpack_from(fmt, data, offset=i)
            if skip_nans and (np.isnan(x) or np.isnan(y) or np.isnan(z)):
                continue
            yield (x, y, z)

def create_cloud_xyz32(points):
    """
    Creates a PointCloud2 message from a NumPy array of XYZ points.
    
    :param points: Numpy array of points (N x 3)
    :return: PointCloud2 message
    """
    cloud = PointCloud2()
    cloud.header.frame_id = "map"  # Change as needed
    cloud.height = 1
    cloud.width = len(points)
    cloud.fields = [
        create_point_field("x", 0),
        create_point_field("y", 4),
        create_point_field("z", 8)
    ]
    cloud.is_bigendian = False
    cloud.point_step = 12  # 4 bytes each for x, y, z
    cloud.row_step = cloud.point_step * cloud.width
    cloud.is_dense = True
    cloud.data = np.asarray(points, dtype=np.float32).tobytes()
    return cloud

def create_point_field(name, offset):
    """
    Helper function to create a PointField for PointCloud2 messages.
    
    :param name: Name of the field
    :param offset: Byte offset for the field
    :return: PointField
    """
    from sensor_msgs.msg import PointField
    field = PointField()
    field.name = name
    field.offset = offset
    field.datatype = PointField.FLOAT32
    field.count = 1
    return field


def pcl_to_array(cloud):
    """
    Converts an Open3D PointCloud object to a NumPy array.
    
    Args:
        cloud: open3d.geometry.PointCloud object
    
    Returns:
        np.ndarray: A NumPy array of shape (N, 3), where N is the number of points.
                    Each row represents (x, y, z) coordinates of a point.
    """
    # Convert Open3D Vector3dVector to a NumPy array
    points = np.asarray(cloud.points)
    
    # Ensure it’s in the expected Nx3 format (x, y, z)
    if points.shape[1] < 3:
        raise ValueError("The input PointCloud does not have enough dimensions (x, y, z).")
    
    return points[:, :3]  # Return only x, y, z