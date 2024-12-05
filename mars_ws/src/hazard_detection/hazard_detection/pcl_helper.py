import numpy as np
import pcl
from sensor_msgs.msg import PointCloud2
import struct

def ros_to_pcl(ros_cloud):
    """
    Converts a ROS PointCloud2 message to a PCL PointCloud.
    
    :param ros_cloud: ROS PointCloud2 message
    :return: PCL PointCloud
    """
    points_list = []

    for point in read_points(ros_cloud, skip_nans=True):
        points_list.append([point[0], point[1], point[2]])

    pcl_data = pcl.PointCloud(points_list, dtype=np.float32)
    #pcl_data.from_array(np.array(points_list, dtype=np.float32))
    return pcl_data

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
    Converts a PCL PointCloud object to a NumPy array.
    Args:
        cloud: pcl.PointCloud object
    
    Returns:
        np.ndarray: A NumPy array of shape (N, 3), where N is the number of points.
                    Each row represents (x, y, z) coordinates of a point.
    """
    # Extract points from the PCL point cloud
    points = np.array(cloud.to_array())
    
    # Ensure it’s in the expected Nx3 format (x, y, z)
    if points.shape[1] < 3:
        raise ValueError("The input PCL point cloud does not have enough dimensions (x, y, z).")
    
    return points[:, :3]  # Return only x, y, z