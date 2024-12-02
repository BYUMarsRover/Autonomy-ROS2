#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from rover_msgs.msg import Heartbeat, HeartbeatStatusRover
from builtin_interfaces.msg import Time
from . import parameters as p


class RoverHeartbeat(Node):
    def __init__(self):
        super().__init__('rover_heartbeat')
        self.last_received = None
        self.sub_heartbeat = self.create_subscription(
            Heartbeat, "/heartbeat_base", self.update_elapsed_time, 10
        )
        self.pub_heartbeat = self.create_publisher(
            Heartbeat, "/heartbeat_rover", 10
        )
        self.pub_heartbeat_status = self.create_publisher(
            HeartbeatStatusRover, "/heartbeat_status_rover", 10
        )
        self.timer = self.create_timer(1.0 / p.RATE, self.ping_and_publish)

    def update_elapsed_time(self, msg):
        t = self.get_clock().now()
        self.get_logger().info("Received heartbeat from base: t=" + str(msg.current_time))
        self.get_logger().debug("Rover received heartbeat: t=" + str(msg.current_time))
        self.last_received = t

    def ping_and_publish(self):
        heartbeat_msg = Heartbeat()
        heartbeat_msg.current_time = self.get_clock().now().to_msg()
        self.pub_heartbeat.publish(heartbeat_msg)

        if self.last_received is not None:
            elapsed_time = (self.get_clock().now() - self.last_received).nanoseconds / 1e9
            status_msg = HeartbeatStatusRover()
            status_msg.elapsed_time = elapsed_time
            self.pub_heartbeat_status.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    heartbeat = RoverHeartbeat()
    rclpy.spin(heartbeat)
    heartbeat.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
