#!/usr/bin/env python3
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from mapviz_tf.lat_lon_meter_convertor import LatLonConvertor

class WaypointTranslator(Node):
    def __init__(self):
        super().__init__('waypoint_clicker')  # Initialize the node with a name
        self.subscription = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.handle_click_callback,
            10  # QoS history depth
        )
        self.result_publisher = self.create_publisher(PointStamped, '/clicked_lat_lon', 10)
        self.convertor = LatLonConvertor()

    def handle_click_callback(self, msg):
        self.get_logger().info(f'Received point: {msg.point}')
        position = self.convertor.convert_to_latlon(msg.point.x, msg.point.y)

        self.get_logger().info(f'Converted to lat: {position["lat"]}, lon: {position["lon"]}')

        result_msg = PointStamped()
        result_msg.header = msg.header
        result_msg.point.x = position["lon"]
        result_msg.point.y = position["lat"]

        self.result_publisher.publish(result_msg)

def spin_in_background(node):
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

def main(args=None):
    rclpy.init()
    waypoint_translator = WaypointTranslator()
    thread = threading.Thread(target=spin_in_background, args=(waypoint_translator,))
    thread.start()
    thread.join()

if __name__ == '__main__':
    main()
