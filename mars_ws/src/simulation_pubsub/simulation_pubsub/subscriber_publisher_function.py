import rclpy
from rclpy.node import Node
from rover_msgs.msg import IWCMotors
from geometry_msgs.msg import Twist

r = 0.15  # Wheel radius (meters)
L = 0.9   # Distance between wheels (meters)

class MinimalPubSub(Node):

    def __init__(self):
        super().__init__('minimal_pubsub')

        # Subscriber
        self.subscription = self.create_subscription(
            IWCMotors,
            'mobility/teleop_drive_cmds',
            self.listener_callback,
            10)
        self.subscription  # Prevent unused variable warning

        # Publisher
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: right={msg.right_middle_speed}, left={msg.left_middle_speed}')

        # Convert IWCMotors message to Twist
        new_msg = Twist()
        if msg.right_middle_dir == 0:
            sim_right_middle_speed = -msg.right_middle_speed
        else:
            sim_right_middle_speed = msg.right_middle_speed
        if msg.left_middle_dir == 0:
            sim_left_middle_speed = -msg.left_middle_speed
        else:
            sim_left_middle_speed = msg.left_middle_speed

        # Prevent errors should the input ever exceed 255 for some reason or if there is a problem with the data
        sim_right_middle_speed = max(-255, min(255, sim_right_middle_speed))
        sim_left_middle_speed = max(-255, min(255, sim_left_middle_speed))
        if msg.right_middle_speed is None or msg.left_middle_speed is None:
            self.get_logger().warn("Received None value for motor speeds, skipping computation.")
            return
        new_msg.linear.x = (sim_right_middle_speed + sim_left_middle_speed) * r / 2
        new_msg.angular.z = (sim_right_middle_speed - sim_left_middle_speed) * r / L

        self.publisher_.publish(new_msg)
        self.get_logger().info(f'Published linear.x: {new_msg.linear.x}, angular.z: {new_msg.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    node = MinimalPubSub()
    rclpy.spin(node)  # Use instance, not class
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
