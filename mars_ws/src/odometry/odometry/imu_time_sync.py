import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuTimeSyncNode(Node):
    def __init__(self):
        super().__init__('imu_time_sync_node')
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )
        self.publisher = self.create_publisher(Imu, '/imu', 10)
        self.get_logger().info('IMU Time Sync Node has been started.')

    def imu_callback(self, msg):
        # Update the timestamp to the current time
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = 'zed_camera_link'

        # Publish the updated message
        self.publisher.publish(msg)
        self.get_logger().debug('Published IMU message with updated timestamp.')

def main(args=None):
    rclpy.init(args=args)
    node = ImuTimeSyncNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()