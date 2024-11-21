#!/usr/bin/env python3

import rclpy
import threading
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.time import Time
from std_msgs.msg import Header
from nav_msgs.msg import Path as PathMsg
from geometry_msgs.msg import Point, Pose, Quaternion, PoseStamped
from mapviz_tf.lat_lon_meter_convertor import LatLonConvertor

def spin_in_background():
    executor = rclpy.get_global_executor()
    try:
        executor.spin()
    except ExternalShutdownException:
        pass

class PathToMapviz(Node):

    def __init__(self):
        super().__init__('path_to_mapviz')

        # Subscriber to "/path_planning/smoothed_path"
        self.path_planning_sub = self.create_subscription(PathMsg, "/path_planning/smoothed_path", self.path_planning_callback, 10)

        # Publisher to "/mapviz/path"
        self.pub = self.create_publisher(PathMsg, '/mapviz/path', 10)

        # Pose array to store poses
        self.poses_array = []

        # Latitude and longitude conversion utility
        self.latlonconv = LatLonConvertor()
        self.meters_per_degree_latitude, self.meters_per_degree_longitude = self.latlonconv.get_meters_per_degree_lat_lon()

    def clean(self):
        """
        This clears the existing path.
        """
        self.poses_array.clear()

    def path_planning_callback(self, msg):
        """
        Callback to process and publish the path.
        """
        self.get_logger().info("New path received!")

        # Create a fixed orientation for each pose
        quaternion_msg = Quaternion()
        quaternion_msg.x = 0
        quaternion_msg.y = 0
        quaternion_msg.z = 0
        quaternion_msg.w = 1

        # Make a header message
        header_msg = Header()
        header_msg.seq = msg.header.seq
        header_msg.stamp = self.get_clock().now().to_msg()
        header_msg.frame_id = 'map'

        # Prepare the path message
        path_msg = PathMsg()
        path_msg.header = header_msg

        # Convert lat/lon to x/y for each pose in the path
        for pose in msg.poses:
            # Convert lat/lon to meters
            point = self.latlonconv.convert_to_meters(pose.pose.position.y, pose.pose.position.x)
            
            # Populate Point message with converted coordinates
            point_msg = Point(x=point['x'], y=point['y'], z=1501)

            # Populate Pose message
            pose_msg = Pose(position=point_msg, orientation=quaternion_msg)

            # Populate PoseStamped message with header and pose
            pose_stamped_msg = PoseStamped(header=header_msg, pose=pose_msg)

            # Add pose to the array
            self.poses_array.append(pose_stamped_msg)

        # Assign the array of poses to the path message and publish it
        path_msg.poses = self.poses_array
        self.get_logger().info("Publishing path message!")
        self.pub.publish(path_msg)


def main():
    rclpy.init()
    node = PathToMapviz()

    try:
        # Run spin in a separate thread
        spin_thread = threading.Thread(target=spin_in_background)
        spin_thread.start()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join()

if __name__ == '__main__':
    main()
