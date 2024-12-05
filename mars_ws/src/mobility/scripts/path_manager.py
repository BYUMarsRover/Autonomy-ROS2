#!/usr/bin/env python3

"""
Adam Welker     BYU MARS ROVER      MAY 2023
Braden Meyers   ROS2 Conversion     DEC 2024

waypoint manager -- Given a feed of current and desired locations,
will stream the needed distance and Chi angles
"""

import rclpy
from rclpy.node import Node

from rover_msgs.msg import RoverStateSingleton, MobilityAutopilotCommand, MobilityGPSWaypoint2Follow, ZedObstacles
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from std_srvs.srv import SetBool
from utils.GPSTools import *
import utm
import numpy as np


class PathManager(Node):

    avoid_hazards = False

    def __init__(self) -> None:
        super().__init__('path_manager')

        self.current_point = None
        self.desired_point = None
        self.manager_name = "Path Manager"

        # ROS 2 Services
        self.enable_server = self.create_service(SetBool, '/mobility/path_manager/enabled', self.enable)
        self.switch_hazard_avoidance = self.create_service(SetBool, '/mobility/hazard_avoidance/enabled', self.enable_hazard_avoidance)

        # ROS 2 Publishers
        self.autopilot_cmds_pub = self.create_publisher(MobilityAutopilotCommand, '/mobility/autopilot_cmds', 10)
        self.debug_pub = self.create_publisher(String, '/mobility/PathManagerDebug', 10)

        self.publish_debug("[__init__] ENTER")

        # ROS 2 Subscribers
        self.gps_waypoint_2_follow_sub = self.create_subscription(
            MobilityGPSWaypoint2Follow,
            '/mobility/waypoint2follow',
            self.waypoint_2_follow_callback,
            10
        )
        self.rover_state_singleton_sub = self.create_subscription(
            RoverStateSingleton,
            '/odometry/rover_state_singleton',
            self.rover_state_singleton_callback,
            10
        )
        self.zed_obstacles_sub = self.create_subscription(
            ZedObstacles,
            '/zed/obstacles',
            self.set_zed_obstacles,
            10
        )

        # Hazard avoidance parameters
        self.roll = None
        self.pitch = None
        self.yaw = None
        self.obstacles = []

        self.get_logger().warn("Waypoint_Manager Initialized!")
        self.publish_debug("[__init__] EXIT")

    def publish_debug(self, message: str):
        """Publish debug messages."""
        self.debug_pub.publish(String(data=message))
        self.get_logger().info(message)

    # Subscriber callbacks
    def rover_state_singleton_callback(self, msg: RoverStateSingleton):
        curr_lat = msg.gps.latitude
        curr_lon = msg.gps.longitude
        curr_elv = msg.gps.altitude
        self.roll = msg.map_roll
        self.pitch = msg.map_pitch
        self.yaw = msg.map_yaw

        self.publish_debug("[rover_state_singleton_callback] Setting self.current_point")
        self.current_point = GPSCoordinate(curr_lat, curr_lon, curr_elv)

        self.update_autopilot_cmd()

    def waypoint_2_follow_callback(self, msg: MobilityGPSWaypoint2Follow):
        try:
            des_lat = msg.latitude
            des_lon = msg.longitude
            des_elv = self.current_point.alt if self.current_point else 0.0

            self.publish_debug("[waypoint_2_follow_callback] Setting self.desired_point")
            self.desired_point = GPSCoordinate(des_lat, des_lon, des_elv)

            self.update_autopilot_cmd()
        except Exception as e:
            self.get_logger().error(f"Exception in waypoint_2_follow_callback: {e}")

    def set_zed_obstacles(self, msg: ZedObstacles):
        self.publish_debug("[set_zed_obstacles] ENTER")
        rover_heading = self.get_rover_heading_from_orientation()

        if self.current_point is not None:
            try:
                self.obstacles = []
                for i in range(len(msg.x_coord)):
                    rel_x = msg.x_coord[i] * np.cos(rover_heading) + msg.y_coord[i] * np.sin(rover_heading)
                    rel_y = -1 * msg.x_coord[i] * np.sin(rover_heading) + msg.y_coord[i] * np.cos(rover_heading)
                    self.obstacles.append((rel_x, rel_y))
            except Exception as e:
                self.get_logger().error(f"Exception in set_zed_obstacles: {e}")

        self.update_autopilot_cmd()
        self.publish_debug("[set_zed_obstacles] EXIT")

    def update_autopilot_cmd(self):
        self.publish_debug("[update_autopilot_cmd] ENTER")

        if self.current_point and self.desired_point:
            self.publish_debug("[update_autopilot_cmd] Calculating autopilot commands")
            self.chi_rad, chi_deg = GPSTools.course_angle_between_lat_lon(self.current_point, self.desired_point)
            self.distance = GPSTools.distance_between_lat_lon(self.current_point, self.desired_point)

            if self.avoid_hazards and self.obstacles:
                self.publish_debug("[update_autopilot_cmd] Executing potential fields")
                self.potential_fields()

            self.autopilot_cmd.distance_to_target = self.distance
            self.autopilot_cmd.course_angle = self.chi_rad

        self.publish_debug("[update_autopilot_cmd] EXIT")

    def enable_hazard_avoidance(self, request: SetBool.Request, response: SetBool.Response):
        self.avoid_hazards = request.data
        response.success = True
        response.message = f'Hazard Avoidance is now {"ON" if self.avoid_hazards else "OFF"}'
        return response

    def get_rover_heading_from_orientation(self):
        return self.yaw

    def potential_fields(self):
        # Placeholder for potential fields logic
        pass


def main(args=None):
    rclpy.init(args=args)
    node = PathManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down PathManager")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
