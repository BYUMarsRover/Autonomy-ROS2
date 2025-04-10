import numpy as np
import pcl
from sensor_msgs.msg import PointCloud2
import struct
import open3d as o3d

def ros_to_pcl_and_transform(ros_cloud, transformation_point, name):
    """
    Converts a ROS PointCloud2 message to an Open3D PointCloud and transforms the points so origin is from the start of the bounding box.
     
    :param ros_cloud: ROS PointCloud2 message
    :return: Open3D PointCloud
    """
    points_list = []

    #TODO: Account for the slight angle of the LIDAR and ZED when transforming the points
    if name == "lidar":
        # Extract points from ROS PointCloud2 message
        # Transform points to rover frame, x-forward, y-right, z-down
        # And also transform the points so origin is from the start of the bounding box
        for point in read_points(ros_cloud, skip_nans=True):
            points_list.append([point[2]-transformation_point[0],   #X axis in the rover frame is Z in the LIDAR frame
                                point[1]-transformation_point[1],   #Y axes are the same
                               -point[0]-transformation_point[2]])  #Z axis in the rover frame is -X in the LIDAR frame
                                                                    #transfromation point in rover frame is the start of the bounding box
    elif name == 'zed':
        # Extract points from ROS PointCloud2 message
        # Transform points to rover frame, x-forward, y-right, z-down
        # Also Include 8 degree rotation around y axis
        # And also transform the points so origin is from the start of the bounding box 

        # Create a rotation matrix for 8 degrees around the y-axis
        angle = np.radians(-10)  # Convert degrees to radians
        R = np.array([[np.cos(angle), 0, np.sin(angle)],
                      [0, 1, 0],
                      [-np.sin(angle), 0, np.cos(angle)]])


        for point in read_points(ros_cloud, skip_nans=True):
            point = [point[0],   #X axes are the same
                    -point[1],   #Y axis in the rover frame is -Y in the ZED frame
                    -point[2]]   #Z axis in the rover frame is -Z in the ZED frame
                                                        
            # Apply the rotation
            rotated_point = np.dot(R, point)

            # Apply the translation
            rotated_point[0] += transformation_point[0]
            rotated_point[1] += transformation_point[1]
            rotated_point[2] += transformation_point[2]

            # Append the rotated point to the list
            points_list.append(rotated_point)

    # Convert to NumPy array
    np_points = np.array(points_list, dtype=np.float32)

    # Create Open3D PointCloud
    pcl_data = o3d.geometry.PointCloud()
    pcl_data.points = o3d.utility.Vector3dVector(np_points)

    # # Generate the fake point cloud
    # fake_pc = generate_fake_point_cloud()
    # return fake_pc

        # Check if the cloud has more than 3 points
    if len(pcl_data.points) < 3:
        return pcl_data
    else:
          # Visualize the point cloud
        # visualize_point_cloud(pcl_data)
        return pcl_data


def visualize_point_cloud(pc):
    # Visualize the point cloud

    # Create a coordinate frame with a specified size
    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0, 0, 0])

    # Visualize the point cloud with the axis
    o3d.visualization.draw_geometries([pc, axis])
    # o3d.visualization.draw_geometries([pc])

def generate_fake_point_cloud():
    """
    Generates a fake point cloud with ground and non-ground points.
    Used for testing purposes.
    """
    # Create ground plane points (z = 0)
    ground_plane = np.random.uniform(-5, 5, (1000, 2))  # 1000 points on the ground plane
    ground_plane = np.hstack((np.zeros((ground_plane.shape[0], 1)), ground_plane))

    #Uncomment to test a sloped plane
    # # Generate 1000 points for the ground plane (x, y coordinates)
    # ground_plane = np.random.uniform(-5, 5, (1000, 2))

    # # Define the slope angle in degrees
    # slope_angle_degrees = 17
    # slope_angle_radians =pcl np.radians(slope_angle_degrees)

    # # Compute the z coordinate based on the slope (e.g., along the x-axis)
    # # z = x * tan(slope_angle)
    # z_coordinates = ground_plane[:, 0] * np.tan(slope_angle_radians)

    # # Add the z coordinates to the points
    # ground_plane = np.hstack((ground_plane, z_coordinates.reshape(-1, 1)))

    # Create non-ground points (random z values)
    non_ground = np.random.uniform(-5, 5, (300, 2))  # 300 non-ground points
    heights = np.random.uniform(1, 5, (non_ground.shape[0], 1))  # z values for non-ground
    non_ground = np.hstack((heights, non_ground))

    #Extra non_ground
    non_ground2 = np.random.uniform(0, 2, (300, 2))  # 300 non-ground points
    heights2 = np.random.uniform(-0.5, 3, (non_ground2.shape[0], 1))  # z values for non-ground
    non_ground2 = np.hstack((heights2, non_ground2))

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
    :param skip_nans: Skip NaN and Inf values
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
            if skip_nans and (np.isnan(x) or np.isnan(y) or np.isnan(z) or np.isinf(x) or np.isinf(y) or np.isinf(z)):
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