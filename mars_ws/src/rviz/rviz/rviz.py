import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
from rover_msgs.msg import HazardArray


from std_msgs.msg import String


class Rviz_Node(Node):

    def __init__(self):
        super().__init__('rviz_node')

        #Publishers
        self.rviz_haz_pub = self.create_publisher(MarkerArray, '/rviz_hazards', 10)
        # self.publisher_ = self.create_publisher(String, 'topic', 10)
        # timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.i = 0

        #Subscribers
        self.subscription = self.create_subscription(HazardArray, '/hazards', self.hazards_callback, 10)
        # self.subscription  # prevent unused variable warning

    # def timer_callback(self):
    #     msg = String()
    #     msg.data = 'Hello World: %d' % self.i
    #     self.publisher_.publish(msg)
    #     self.get_logger().info('Publishing: "%s"' % msg.data)
    #     self.i += 1

    def hazards_callback(self, msg):
        self.get_logger().info('Hazard(s) Detected: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)
    viz_node = Rviz_Node()
    rclpy.spin(viz_node)
    viz_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
