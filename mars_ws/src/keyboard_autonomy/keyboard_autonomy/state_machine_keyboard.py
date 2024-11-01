#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import cv2

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rover_msgs.msg import ObstacleInfo, WaypointNav, NavStatus, DynamixelState, ZedObstacles
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class KeyboardStateMachine(Node):
    def __init__(self):
        super().__init__('autonomy_state_machine')
        self.img = None
        self.bridge = CvBridge()
        self.sub_keyboard_img = self.create_subscription(Image, '/camera/rgb/keyboard_cam/image_raw',
                                                         self.image_callback, 10)

    def image_callback(self, msg):
        try:
            self.img = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")

    def openCV(self):
        if self.img is not None:
            cv2.imshow("Annotated Image", self.img)
            if cv2.waitKey(10) == 13:
                rclpy.shutdown()
                cv2.destroyAllWindows()


def main(args=None):
    """
     Main function for the state machine

     Continuous loop that publishes status and checks for updates from the GUI

     FIXME this is probably where we can fix the fact that it finishes a loop before stopping once the GUI tells it to
     """
    rclpy.init(args=args)
    keyboard_state_machine = KeyboardStateMachine()
    rate = keyboard_state_machine.create_rate(10)

    while rclpy.ok():
        keyboard_state_machine.openCV()
        rclpy.spin_once(keyboard_state_machine)
        rate.sleep()

    keyboard_state_machine.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
