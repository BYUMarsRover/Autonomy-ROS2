import math
from geographic_msgs.msg import GeoPose
from geometry_msgs.msg import Quaternion
import utm


def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr
    return q


# I think we might just want to grab these from tf transformations
def euler_from_quaternion(q: Quaternion):
    """
    Convert a quaternion into euler angles
    taken from: https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
    """
    t0 = +2.0 * (q.w * q.x + q.y * q.z)
    t1 = +1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (q.w * q.y - q.z * q.x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (q.w * q.z + q.x * q.y)
    t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z


def latLonYaw2Geopose(latitude: float, longitude: float, yaw: float = 0.0) -> GeoPose:
    """
    Creates a geographic_msgs/msg/GeoPose object from latitude, longitude and yaw
    """
    geopose = GeoPose()
    geopose.position.latitude = latitude
    geopose.position.longitude = longitude
    geopose.orientation = quaternion_from_euler(0.0, 0.0, yaw)
    return geopose


def geopose2LatLonYaw(geopose: GeoPose) -> tuple[float, float, float]:
    """
    Extracts latitude, longitude and yaw from a geographic_msgs/msg/GeoPose object
    """

    return (
        geopose.position.latitude,
        geopose.position.longitude,
        euler_from_quaternion(geopose.orientation)[2],
    )


def latLon2Meters(lat1, lon1, lat2, lon2):
    """
    Convert GPS coordinates to meters using the UTM library
    """

    # Convert GPS coordinates to UTM
    utm1 = utm.from_latlon(lat1, lon1)
    utm2 = utm.from_latlon(lat2, lon2)

    # Calculate the distance between the two points
    distance = ((utm2[0] - utm1[0]) ** 2 + (utm2[1] - utm1[1]) ** 2) ** 0.5

    return distance


def meters2LatLon(lat, lon, x_offset, y_offset):
    """
    Convert meters to GPS coordinates using the UTM library
    """

    # Convert GPS coordinates to UTM
    utm1 = utm.from_latlon(lat, lon)

    # Calculate the new UTM coordinates
    utm2 = (utm1[0] + x_offset, utm1[1] + y_offset)

    # Convert UTM coordinates back to GPS
    lat2, lon2 = utm.to_latlon(utm2[0], utm2[1], utm1[2], utm1[3])

    return lat2, lon2
