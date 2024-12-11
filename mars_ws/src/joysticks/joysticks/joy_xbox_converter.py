import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from rover_msgs.msg import Xbox
from joy_utils import joy_msg_to_xbox_msg

class JoyToXboxConverter(Node):
    def __init__(self):
        super().__init__('JtXC')
        
        self.declare_parameter('joy_topic', 'joy')
        self.declare_parameter('output_topic', None)
        self.declare_parameter('deadzone', 0.05)
        
        joy_topic = self.get_parameter('joy_topic').value
        output_topic = self.get_parameter('output_topic').value
        deadzone = self.get_parameter('deadzone').value
        
        if output_topic is None:
            output_topic = joy_topic + "_processed"
        
        if isinstance(deadzone, str):
            deadzone = float(deadzone)
        
        self.deadzone = deadzone
        
        self.sub_joy = self.create_subscription(
            Joy,
            joy_topic,
            self.process_joy,
            10)
        self.pub_xbox = self.create_publisher(Xbox, output_topic, 10)

    def process_joy(self, joy_msg):
        self.pub_xbox.publish(joy_msg_to_xbox_msg(joy_msg, self.deadzone))

def main(args=None):
    rclpy.init(args=args)
    jtxc = JoyToXboxConverter()
    rclpy.spin(jtxc)
    jtxc.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
