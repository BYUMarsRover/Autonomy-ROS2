import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Bool

class GStreamer2ROS2Node(Node):
    '''
    :author: Ethan Blaylock
    :date: January 2025

    ROS2 node that takes a video on GStreamer and republishes on ROS2 topic

    Publishes:
        - /image_raw (sensor_msgs/msg/Image)
    '''
    def __init__(self):

        super().__init__('gstreamer2ros2')
        self.get_logger().info("GStreamer2ROS2Node started")
        self.gstreamer_pipeline = 'udpsrc address=192.168.1.111 port=5010 caps="application/x-rtp,media=video,encoding-name=H265,payload=96" ! rtph265depay ! h265parse ! avdec_h265 ! videoconvert ! appsink'
        self.ros_topic = 'image_raw'
        self.frame_rate = 10.0

        # Publisher
        self.publisher = self.create_publisher(Image, self.ros_topic, 10)
        '''
        Publisher to the "/image_raw" topic with the message type Image.
        '''

        # Timer
        timer_period = 1.0 / self.frame_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # Initialize GStreamer
        try:
            self.cap = cv2.VideoCapture(self.gstreamer_pipeline, cv2.CAP_GSTREAMER)
        except:
            self.get_logger().info("Failed to create GStreamer pipeline2!")
            self.destroy()
            raise BrokenPipeError
        
        if not self.cap:
            self.get_logger().info("Failed to create GStreamer pipeline!")
            raise BrokenPipeError
        elif not self.cap.isOpened():
            self.get_logger().info("Failed to open GStreamer pipeline!")
            self.destroy()
            raise BrokenPipeError
        self.get_logger().info("Pipeline is open, yay!")

    def timer_callback(self):
        self.get_logger().info("Timer callback")
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().info("Failed to capture frame")
            return
        
        # Try to convert frame to ROS 2 Image message and publish
        try:
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher.publish(ros_image)
        except Exception as e:
            self.get_logger().info(f"Failed to publish image: {str(e)}")

    def destroy(self):
        # Release resources
        self.cap.release()

def main(args=None):
    rclpy.init(args=args)

    try:
        node = GStreamer2ROS2Node()
        rclpy.spin(node)
    except BrokenPipeError:
        rclpy.shutdown()
    except KeyboardInterrupt:
        node.destroy()
    finally:
        pass

if __name__ == '__main__':
    main()