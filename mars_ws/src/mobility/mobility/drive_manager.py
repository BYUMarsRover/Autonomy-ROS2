#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rover_msgs.msg import MobilityDriveCommand, MobilityVelocityCommands
from rover_msgs.srv import SetFloat32
from std_srvs.srv import SetBool
import numpy as np

class DriveManager(Node):
    def __init__(self):
        super().__init__('drive_manager')

        # Subscribers
        self.vel_cmds_sub = self.create_subscription(
            MobilityVelocityCommands, 
            '/mobility/rover_vel_cmds', 
            self.vel_cmds_callback, 
            10
        )

        # Publishers
        self.wheel_vel_cmds_pub = self.create_publisher(
            MobilityDriveCommand, 
            '/mobility/wheel_vel_cmds', 
            10
        )

        # Service
        self.enable_server = self.create_service(
            SetBool, 
            '/mobility/drive_manager/enabled', 
            self.enable
        )
        self.set_turn_constant_service = self.create_service(
            SetFloat32,
            '/mobility/drive_manager/set_turn_constant',
            self.set_turn_constant
        )
        self.set_speed_service = self.create_service(
            SetFloat32,
            '/mobility/drive_manager/set_speed',
            self.set_speed
        )

        # Parameters #TODO: this node is not getting these parameters properly form the params file being passed in by the launch file
        self.declare_parameter('cmd_lb', 0.01) # Wheel Command Lower Bound
        self.declare_parameter('max_speed', 3.0) # Maximum Speed

        self.cmd_lb = self.get_parameter('cmd_lb').get_parameter_value().double_value
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value

        # Attributes
        self.r = 0.8382  # wheel radius (meters)
        self.B = 0.1335  # wheel base distance (meters)
        self.k = 0.5     # parameter for sigmoid function
        self.turn_constant = 20.0
        self.rover_cmd = MobilityDriveCommand()
        self.enabled = False

        self.get_logger().info(f"Drive Manager initialized!")

    def set_turn_constant(self, request, response):
        self.turn_constant = request.data
        response.success = True
        response.message = f"Drive Manager: Turn Constant set to {self.turn_constant}"
        return response
    
    def set_speed(self, request, response):
        if request.data > 10.0:
            response.success = False
            response.message = f"Drive Manager: Max Speed cannot be greater than 10.0"
            return response
        else:
            self.max_speed = 10.1 - request.data
            response.success = True
            response.message = f"Drive Manager: Max Speed set to {request.data}"
        return response

    def vel_cmds_callback(self, msg): # NOTE: HERE
        #Postive angle is right turn, negative angle is left turn (NED coordinate frame)
        u_cmd = msg.u_cmd
        omega_cmd = msg.omega_cmd

        if u_cmd == 0 and omega_cmd == 0:
            rw_speed = 0.0
            lw_speed = 0.0
        else:
            v_l = u_cmd + omega_cmd * self.B / 2 * self.turn_constant # NOTE: turn constant was a quick fix
            v_r = u_cmd - omega_cmd * self.B / 2 * self.turn_constant
            # self.get_logger().info(f"IN: vel_cmds_callback, v_l/v_r: {v_l}/{v_r}")
            psidot_Ld = v_l / self.r
            psidot_Rd = v_r / self.r
            lw_speed = self.piecewise_sigmoid(psidot_Ld)
            rw_speed = self.piecewise_sigmoid(psidot_Rd)
            # self.get_logger().info(f"IN: vel_cmds_callback, lw/rw: {lw_speed}/{rw_speed}")

        self.rover_cmd.rw = float(rw_speed)
        self.rover_cmd.lw = float(lw_speed)
        self.publish_rover_cmd()

    def publish_rover_cmd(self):
        if not self.enabled:
            self.rover_cmd = MobilityDriveCommand()

        self.wheel_vel_cmds_pub.publish(self.rover_cmd)

    def enable(self, request, response):
        self.enabled = request.data
        response.success = True
        response.message = f"Drive Manager: {'ENABLED' if self.enabled else 'DISABLED'}"
        return response

    def piecewise_sigmoid(self, x):
        # self.get_logger().info(f"IN: piecewise_sigmoid, x: {x}")
        m = (1 - self.cmd_lb) / self.max_speed
        if x < -self.max_speed:
            return -1
        elif -self.max_speed <= x < 0:
            return -1 + m * (x + self.max_speed)
        elif x == 0:
            return 0
        elif 0 < x <= self.max_speed:
            return self.cmd_lb + m * x
        elif x > self.max_speed:
            return 1
        else:
            self.get_logger().error(f"Invalid input to piecewise_sigmoid: {x}")
            return 0

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
