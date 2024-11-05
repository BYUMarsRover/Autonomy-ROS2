#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from sensor_msgs.msg import NavSatFix
from rover_msgs.msg import RoverStateSingleton, AutonomyTaskInfo
from rover_msgs.srv import AutonomyWaypoint, AutonomyWaypointResponse
from ublox_read_2.msg import PositionVelocityTime

ROS_RATE = 15

class GPSPublisher(Node):
    def __init__(self):
        super().__init__("gps_to_mapviz")

        # Publishers
        self.rover_filtered_gps_pub = self.create_publisher(NavSatFix, "/GPSFix_rover_filtered", 1)
        self.rover_unfiltered_gps_pub = self.create_publisher(NavSatFix, "/GPSFix_rover_unfiltered", 1)
        self.base_gps_pub = self.create_publisher(NavSatFix, "/GPSFix_base", 1)
        self.gps_plotter_pub = self.create_publisher(NavSatFix, "/GPS_waypoint_plotter", 1)

        # Service
        self.gps_plotter_srv = self.create_service(AutonomyWaypoint, '/GPS_waypoint_plotter', self.set_all_tasks_callback)

        # Subscriptions
        self.rover_state_singleton_sub = self.create_subscription(
            RoverStateSingleton,
            "/odometry/rover_state_singleton",
            self.singleton_callback,
            1
        )

        self.base_sub = self.create_subscription(
            PositionVelocityTime,
            "/base/PosVelTime",
            self.base_callback,
            1
        )

        self.rover_state_singleton = RoverStateSingleton()
        self.base_gps = PositionVelocityTime()

        self.get_logger().info("GPS Publisher Node initialized.")

    def singleton_callback(self, msg):
        self.rover_state_singleton = msg

    def base_callback(self, msg):
        self.base_gps = msg

    def publish_gps_data(self):
        yaw = self.rover_state_singleton.map_yaw
        self._publish_base_gps_data(self.base_gps_pub, self.base_gps)
        self._publish_rover_gps_data(
            self.rover_filtered_gps_pub, self.rover_state_singleton.filter_gps, yaw
        )
        self._publish_rover_gps_data(
            self.rover_unfiltered_gps_pub, self.rover_state_singleton.gps, yaw
        )

    def _publish_rover_gps_data(self, gps_publisher, gps, yaw):
        NavSatFix_msg = NavSatFix(latitude=gps.latitude, longitude=gps.longitude)
        gps_publisher.publish(NavSatFix_msg)

    def _publish_base_gps_data(self, gps_publisher, posVelTime):
        NavSatFix_msg = NavSatFix(
            latitude=posVelTime.lla[0], longitude=posVelTime.lla[1]
        )
        gps_publisher.publish(NavSatFix_msg)

    def set_all_tasks_callback(self, request, response):
        self.get_logger().info('Adding waypoints to the queue')
        tasks = request.task_list
        for task_info in tasks:
            gps_msg = NavSatFix(latitude=task_info.latitude, longitude=task_info.longitude)
            self.gps_plotter_pub.publish(gps_msg)
        self.get_logger().info('Waypoints successfully added.')

        response.success = True
        response.message = 'Adding waypoints was successful'
        return response

def main(args=None):
    rclpy.init(args=args)
    gps_publisher = GPSPublisher()
    rate = gps_publisher.create_rate(ROS_RATE)

    try:
        while rclpy.ok():
            rclpy.spin_once(gps_publisher)
            gps_publisher.publish_gps_data()
            rate.sleep()
    except ExternalShutdownException:
        pass
    finally:
        gps_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()



















# #!/usr/bin/env python3

# import threading
# import rclpy
# from rclpy.node import Node
# from rclpy.executors import ExternalShutdownException

# import numpy as np

# from sensor_msgs.msg import NavSatFix
# from rover_msgs.msg import RoverStateSingleton, AutonomyTaskInfo
# from rover_msgs.srv import AutonomyWaypoint, AutonomyWaypointResponse
# from ublox_read_2.msg import PositionVelocityTime

# def spin_in_background():
#     executor = rclpy.get_global_executor()
#     try:
#         executor.spin()
#     except ExternalShutdownException:
#         pass

# ROS_RATE = 15

# class GPSPublisher(Node):
#     def __init__(self):
#         self.rover_filtered_gps_pub = self.create_publisher(NavSatFix, "/GPSFix_rover_filtered", 1)
#         self.rover_unfiltered_gps_pub = self.create_publisher(NavSatFix, "/GPSFix_rover_unfiltered", 1)
#         self.base_gps_pub = self.create_publisher(NavSatFix, "/GPSFix_base", 1)
#         self.gps_plotter_pub = self.create_publisher(NavSatFix, "/GPS_waypoint_plotter", 1)

#         self.gps_plotter_srv = self.create_service(AutonomyWaypoint, '/GPS_waypoint_plotter', self.set_all_tasks_callback)

#         self.rover_state_singleton_sub = self.create_subscription(
#             RoverStateSingleton,
#             "/odometry/rover_state_singleton",
#             self.singleton_callback,
#             1
#         )

#         self.base_sub = self.create_subscription(
#             PositionVelocityTime,
#             "/base/PosVelTime",
#             self.base_callback,
#             1
#         )

#         self.rover_state_singleton = RoverStateSingleton()
#         self.base_gps = PositionVelocityTime()

#         print("Setting done for GPS")

#     def singleton_callback(self, msg):
#         self.rover_state_singleton = msg

#     def base_callback(self, msg):
#         self.base_gps = msg

#     def publish_gps_data(self):
#         yaw = self.rover_state_singleton.map_yaw
#         self._publish_base_gps_data(self.base_gps_pub, self.base_gps)
#         self._publish_rover_gps_data(
#             self.rover_filtered_gps_pub, self.rover_state_singleton.filter_gps, yaw
#         )
#         self._publish_rover_gps_data(
#             self.rover_unfiltered_gps_pub, self.rover_state_singleton.gps, yaw
#         )

#     def _publish_rover_gps_data(self, gps_publisher, gps, yaw):
#         NavSatFix_msg = NavSatFix(latitude=gps.latitude, longitude=gps.longitude)
#         gps_publisher.publish(NavSatFix_msg)

#     def _publish_base_gps_data(self, gps_publisher, posVelTime):
#         NavSatFix_msg = NavSatFix(
#             latitude=posVelTime.lla[0], longitude=posVelTime.lla[1]
#         )
#         gps_publisher.publish(NavSatFix_msg)

#     def set_all_tasks_callback(self, task_list:AutonomyWaypoint) -> AutonomyWaypointResponse:
#         print('in set_all_tasks_callback')
#         tasks: list[AutonomyTaskInfo] = task_list.task_list
#         for task_info in tasks:
#             gps_msg = NavSatFix(latitude=task_info.latitude, longitude=task_info.longitude)
#             self.gps_plotter_pub.publish(gps_msg)
#         print('Exiting set_all_tasks_callback')

#         response = AutonomyWaypointResponse()
#         response.success = True
#         response.message = 'Adding waypoints was successful'
#         return response

# if __name__ == "__main__":
#     rclpy.init()
#     t = threading.Thread(target=spin_in_background)
#     t.start()
#     node = rclpy.create_node("gps_to_mapviz")
#     rclpy.get_global_executor().add_node(node)
#     rate = node.create_rate(ROS_RATE)
#     gps_pub = GPSPublisher()
#     while rclpy.ok():
#         gps_pub.publish_gps_data()
#     t.join()