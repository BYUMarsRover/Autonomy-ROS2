#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rover_msgs.msg import MobilityDriveCommand, MobilityVelocityCommands
from std_srvs.srv import SetBool
import numpy as np

class DriveManager(Node):
    def __init__(self):
        super().__init__('drive_manager')

        # ROS 2 Subscribers
        self.vel_cmds_sub = self.create_subscription(
            MobilityVelocityCommands, 
            '/mobility/rover_vel_cmds', 
            self.vel_cmds_callback, 
            10
        )

        # ROS 2 Publishers
        self.wheel_vel_cmds_pub = self.create_publisher(
            MobilityDriveCommand, 
            '/mobility/wheel_vel_cmds', 
            10
        )

        # ROS 2 Service
        self.enable_server = self.create_service(
            SetBool, 
            '/mobility/drive_manager/enabled', 
            self.enable
        )

        # Parameters
        self.declare_parameter('cmd_lb', 0.0)
        self.declare_parameter('max_speed', 1.0)

        self.cmd_lb = self.get_parameter('cmd_lb').value
        self.max_speed = self.get_parameter('max_speed').value

        # Attributes
        self.r = 0.8382  # wheel radius (meters)
        self.B = 0.1335  # wheel base distance (meters)
        self.k = 0.5     # parameter for sigmoid function
        self.rover_cmd = MobilityDriveCommand()
        self.enabled = False
        self.manager_name = "Drive Manager"

        self.get_logger().info(f"{self.manager_name} initialized")

    def vel_cmds_callback(self, msg):
        u_cmd = msg.u_cmd
        omega_cmd = msg.omega_cmd

        if u_cmd == 0 and omega_cmd == 0:
            rw_speed = 0
            lw_speed = 0
        else:
            v_l = u_cmd - omega_cmd * self.B / 2
            v_r = u_cmd + omega_cmd * self.B / 2
            psidot_Ld = v_l / self.r
            psidot_Rd = v_r / self.r
            rw_speed = self.piecewise_sigmoid(psidot_Rd)
            lw_speed = self.piecewise_sigmoid(psidot_Ld)

        self.rover_cmd.rw = rw_speed
        self.rover_cmd.lw = lw_speed
        self.publish_rover_cmd()

    def publish_rover_cmd(self):
        if not self.enabled:
            self.rover_cmd = MobilityDriveCommand()

        self.wheel_vel_cmds_pub.publish(self.rover_cmd)

    def enable(self, request, response):
        self.enabled = request.data
        response.success = True
        response.message = f"{self.manager_name} is {'enabled' if self.enabled else 'disabled'}"
        self.get_logger().info(response.message)
        return response

    def piecewise_sigmoid(self, x):
        m = (1 - self.cmd_lb) / self.max_speed
        if x < -self.max_speed:
            return -1
        elif -self.max_speed <= x < 0:
            return -1 + m * (x + self.max_speed)
        elif x == 0:
            return 0
        elif 0 < x <= self.max_speed:
            return self.cmd_lb + m * x
        else:  # x > self.max_speed
            return 1

def main(args=None):
    rclpy.init(args=args)
    drive_manager = DriveManager()

    try:
        rclpy.spin(drive_manager)
    except KeyboardInterrupt:
        pass
    finally:
        drive_manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
