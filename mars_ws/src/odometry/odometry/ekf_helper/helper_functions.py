import math

def convert_gps_to_xyz(lat, lon, ref_latitude_deg=40.246105, ref_longitude_deg=-111.647000):
    """
    Convert latitude and longitude to local Cartesian coordinates (x, y) in an East-North-Up (ENU) frame 
    centered at the reference latitude and longitude.
    """
    # Convert degrees to radians
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    ref_lat_rad = math.radians(ref_latitude_deg)
    ref_lon_rad = math.radians(ref_longitude_deg)
    
    # Constants
    earth_radius = 6378137.0  # Earth's radius in meters
    
    # Calculate deltas
    delta_lat = lat_rad - ref_lat_rad
    delta_lon = lon_rad - ref_lon_rad
    
    # Approximate x and y using a local tangent plane
    x = delta_lon * earth_radius * math.cos(ref_lat_rad)  # Longitude scaling by cos(lat)
    y = delta_lat * earth_radius                          # Latitude scaling
    
    return x, y


def convert_xyz_to_gps(x, y, ref_latitude_deg=40.246105, ref_longitude_deg=-111.647000):
    """
    Convert local Cartesian coordinates (x, y) back to latitude and longitude.

    """
    # Convert degrees to radians
    ref_lat_rad = math.radians(ref_latitude_deg)
    ref_lon_rad = math.radians(ref_longitude_deg)   
    # Constants
    earth_radius = 6378137.0  # Earth's radius in meters
    # Calculate deltas
    delta_lat = y / earth_radius
    delta_lon = x / (earth_radius * math.cos(ref_lat_rad))
    # Convert back to radians
    lat_rad = ref_lat_rad + delta_lat
    lon_rad = ref_lon_rad + delta_lon

    # Convert to degrees
    lat_deg = math.degrees(lat_rad)
    lon_deg = math.degrees(lon_rad)

    return lat_deg, lon_deg