import math
from itertools import permutations
from rover_navigation.utils.gps_utils import (
    latLonYaw2Geopose,
    quaternion_from_euler,
    latLon2Meters,
)


def basicPathPlanner(geopose1, geopose2, wp_dist):
    """
    Generate intermediary waypoints in a straight line between two GPS coordinates

    :author: Nelson Durrant
    :date: Mar 2025
    """

    new_wps = []

    # Get starting waypoint GPS coordinates
    start_lat = geopose1.position.latitude
    start_lon = geopose1.position.longitude
    end_lat = geopose2.position.latitude
    end_lon = geopose2.position.longitude

    # Calculate the desire yaw angle for movement
    yaw = math.atan((end_lat - start_lat) / (end_lon - start_lon))
    if end_lon < start_lon:
        yaw += math.pi

    # Calculate the distance between the two points in lat/lon degrees
    distance = latLon2Meters(start_lat, start_lon, end_lat, end_lon)

    # Calculate the number of intermediary waypoints
    num_waypoints = int(math.ceil(distance / wp_dist))

    if num_waypoints != 0:

        # Calculate the step size for each intermediary waypoint
        step_lat = (end_lat - start_lat) / num_waypoints
        step_lon = (end_lon - start_lon) / num_waypoints

        # Generate intermediary waypoints
        for i in range(1, num_waypoints):
            lat = start_lat + i * step_lat
            lon = start_lon + i * step_lon

            geopose = latLonYaw2Geopose(lat, lon, yaw)

            new_wps.append(geopose)

    # Add the original waypoint
    geopose2.orientation = quaternion_from_euler(0.0, 0.0, yaw)
    new_wps.append(geopose2)

    return new_wps


def basicOrderPlanner(legs, fix):
    """
    Brute force the optimal order to complete the task legs (based on distance)

    This is an NP-hard problem, but we deal with such small numbers of legs that we can brute force a 
    basic optimal solution in a reasonable time.

    :author: Nelson Durrant
    :date: Mar 2025
    """

    lowest_cost = float("inf")
    best_order = []

    # Generate all possible permutations of the task legs
    for order in permutations(legs):

        # Calculate the cost of the current order
        fix_geopose = latLonYaw2Geopose(fix.position.latitude, fix.position.longitude)
        leg_geopose = latLonYaw2Geopose(order[0].latitude, order[0].longitude)
        cost = latLon2Meters(
            fix_geopose.position.latitude,
            fix_geopose.position.longitude,
            leg_geopose.position.latitude,
            leg_geopose.position.longitude,
        )

        for i in range(len(order) - 1):

            leg1_geopose = latLonYaw2Geopose(order[i].latitude, order[i].longitude)
            leg2_geopose = latLonYaw2Geopose(
                order[i + 1].latitude, order[i + 1].longitude
            )
            cost += latLon2Meters(
                leg1_geopose.position.latitude,
                leg1_geopose.position.longitude,
                leg2_geopose.position.latitude,
                leg2_geopose.position.longitude,
            )

        # Update the best order
        if cost < lowest_cost:
            lowest_cost = cost
            best_order = order

    return best_order
