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

    def update_elapsed_time(self, msg):
        t = self.get_clock().now()
        self.get_logger().info("Received first heartbeat from base: t=" + str(t))
        self.get_logger().debug("Rover received heartbeat: t=" + str(t))
        self.last_received = t

    def ping_and_publish(self):
        heartbeat_msg = Heartbeat()
        heartbeat_msg.current_time = Time()
        now = self.get_clock().now()
        heartbeat_msg.current_time.sec = now.seconds
        heartbeat_msg.current_time.nanosec = now.nanoseconds

        self.pub_heartbeat.publish(heartbeat_msg)

        if self.last_received is not None:
            elapsed_time = (self.get_clock().now() - self.last_received).nanoseconds / 1e9
            status_msg = HeartbeatStatusRover()
            status_msg.elapsed_time = elapsed_time
            self.pub_heartbeat_status.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    heartbeat = RoverHeartbeat()
    rate = heartbeat.create_rate(p.RATE)

    while rclpy.ok():
        heartbeat.ping_and_publish()
        rclpy.spin_once(heartbeat)
        rate.sleep()

    heartbeat.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
