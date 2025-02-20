#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rover_msgs.msg import IWCMotors, MobilityDriveCommand
from std_srvs.srv import SetBool
# from manager_interface import Manager


class WheelManager(Node):

    def __init__(self):
        super().__init__('wheel_manager')

        # Subscribers
        self.IWC_cmd = IWCMotors()

        self.wheel_vel_cmds_sub = self.create_subscription(
            MobilityDriveCommand,
            '/mobility/wheel_vel_cmds',
            self.wheel_vel_cmds_callback,
            10
        )

        # Publishers
        self.auto_drive_cmds_pub = self.create_publisher(
            IWCMotors,
            '/mobility/auto_drive_cmds',
            10
        )

        # Service
        self.enable_server = self.create_service(
            SetBool,
            '/mobility/wheel_manager/enabled',
            self.enable
        )

        self.enabled = False
        self.timer = self.create_timer(0.1, self.publish_wheel_cmd)

        self.get_logger().info(f"Wheel Manager initialized!")

    def wheel_vel_cmds_callback(self, msg):
        left_dir = self._check_dir(msg.lw)
        right_dir = self._check_dir(msg.rw)

        left_speed = self._check_speed(msg.lw)
        right_speed = self._check_speed(msg.rw)

        # Populate IWC Message
        self.IWC_cmd.right_front_speed = abs(right_speed)
        self.IWC_cmd.right_front_dir = right_dir

        self.IWC_cmd.right_middle_speed = abs(right_speed)
        self.IWC_cmd.right_middle_dir = right_dir

        self.IWC_cmd.right_rear_speed = abs(right_speed)
        self.IWC_cmd.right_rear_dir = right_dir

        self.IWC_cmd.left_front_speed = abs(left_speed)
        self.IWC_cmd.left_front_dir = left_dir

        self.IWC_cmd.left_middle_speed = abs(left_speed)
        self.IWC_cmd.left_middle_dir = left_dir

        self.IWC_cmd.left_rear_speed = abs(left_speed)
        self.IWC_cmd.left_rear_dir = left_dir

    def _check_speed(self, val):
        '''
        Ensures that the val is a float and is between 0-255 
        '''
        return int(abs(val) * 255.)

    def _check_dir(self, val):
        return val >= 0

    def publish_wheel_cmd(self):
        if not self.enabled:
            self.IWC_cmd = IWCMotors()  # Reset the command if disabled
            self.auto_drive_cmds_pub.publish(self.IWC_cmd)
            return

        if self.enabled:
            self.auto_drive_cmds_pub.publish(self.IWC_cmd)

    def enable(self, request, response):
        '''
        Service callback to enable or disable the wheel manager
        '''
        self.enabled = request.data
        response.success = True
        response.message = f"Wheel Manager: {'ENABLED' if self.enabled else 'DISABLED'}"
        return response


def main(args=None):
    rclpy.init(args=args)
    wheel_manager = WheelManager()

    try:
        rclpy.spin(wheel_manager)
    except KeyboardInterrupt:
        pass
    finally:
        wheel_manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
