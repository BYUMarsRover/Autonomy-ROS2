#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rover_msgs.msg import IWCMotors
# TODO: See if we can convert mobility drive enable to a service
from std_msgs import Bool
from geometry_msgs.msg import Twist

class Transition(Node):
    def __init__(self):
        super().__init__('transition')
        
        # ROS Publishers
        self.IWC_control_pub = self.create_publisher(IWCMotors, '/IWC_motorControl', 1)
        self.sim_control_pub = self.create_publisher(Twist, '/mobility/cmd_vel', 1)

        # ROS Subscribers
        self.joy_drive_enabled_sub = self.create_subscription(
            Bool,
            '/mobility/joy_drive_enabled',
            self.joy_drive_enabled_callback,
            1)
        self.teleop_drive_cmds_sub = self.create_subscription(
            IWCMotors,
            '/mobility/teleop_drive_cmds',
            self.teleop_drive_cmds_callback,
            1)
        self.auto_drive_cmds_sub = self.create_subscription(
            IWCMotors,
            '/mobility/auto_drive_cmds',
            self.auto_drive_cmds_callback,
            1)
        
        self.joy_drive_enabled = False

    def joy_drive_enabled_callback(self, msg):
        self.joy_drive_enabled = msg.data

    def teleop_drive_cmds_callback(self, IWC_cmd_msg):
        if self.joy_drive_enabled:
            self.IWC_control_pub.publish(IWC_cmd_msg)

    def auto_drive_cmds_callback(self, IWC_cmd_msg):
        if not self.joy_drive_enabled:
            self.IWC_control_pub.publish(IWC_cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    transition = Transition()
    rclpy.spin(transition)
    transition.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
