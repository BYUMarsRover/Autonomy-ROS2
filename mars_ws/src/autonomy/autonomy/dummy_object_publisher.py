import rclpy
from rclpy.node import Node
from rover_msgs.msg import ObjectDetections, ObjectDetection
import numpy as np

class dummyObjectPublisher(Node):
    def __init__(self):
        super().__init__('dummy_object_publisher')

        self.publisher_ = self.create_publisher(ObjectDetections, '/zed/object_detection', 10)
        timer_period = 0.2  # seconds - 5 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.x = 2.0
        self.y = 3.0
        self.z = -0.8

    def arc_increment(self): # make the object move side to side in an arc around the zed
        self.x = 3*np.cos(3/2*np.pi + np.pi*np.abs(np.sin(np.pi*self.i/100)))
        self.y = -3*np.sin(3/2*np.pi + np.pi*np.abs(np.sin(np.pi*self.i/100)))

    def linear_increment(self): # make the object oscillate linearly in the x direction
        self.y = 0.0
        self.x = np.cos(self.i/100)*5

    def timer_callback(self):
        msg = ObjectDetections()
        obj_msg = ObjectDetection()

        obj_msg.id = int(1)
        obj_msg.label = int(0) # 0 is mallet, 1 is bottle
        obj_msg.confidence = 0.9
        obj_msg.x = self.x
        obj_msg.y = self.y
        obj_msg.z = -0.8

        msg.objects.append(obj_msg)
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'x:{self.x:.3f}, y:{self.y:.3f}, z:{self.z:.3f}')
        self.i += 1
        # self.arc_increment()
        self.linear_increment()


def main(args=None):
    rclpy.init(args=args)

    dummy_object_publisher = dummyObjectPublisher()

    rclpy.spin(dummy_object_publisher)

    dummy_object_publisher.destroy_node()
    rclpy.shutdown()