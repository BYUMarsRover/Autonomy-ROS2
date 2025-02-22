#!/usr/bin/env python3

"""
Adam Welker     BYU MARS ROVER  MAY 2023
Braden Meyers   ROS2 Conversion DEC 2024

autopilot_manager.py -- This class will output linear and angular velocity commands 
for the drive manager given current vs desired heading, as well as distance to target objective
"""

import rclpy
from rclpy.node import Node
import numpy as np

from rover_msgs.msg import MobilityVelocityCommands, MobilityArucoAutopilotCommand
from rover_msgs.srv import SetFloat32
from std_srvs.srv import SetBool

from mobility.utils.wrap import wrap
from mobility.controllers.pid_control import PIDControl


class ArucoAutopilotManager(Node):

    def __init__(self):
        super().__init__('aruco_autopilot_manager')

        # Initialize publishers, subscribers, and services
        self.rover_vel_cmds_pub = self.create_publisher(MobilityVelocityCommands, '/mobility/rover_vel_cmds', 10)
        self.autopilot_cmds_sub = self.create_subscription(
            MobilityArucoAutopilotCommand, '/mobility/aruco_autopilot_cmds', self.aruco_autopilot_cmds_callback, 10)

        self.enable_service = self.create_service(SetBool, '/mobility/aruco_autopilot_manager/enabled', self.enable_callback)
        # self.speed_service = self.create_service(SetFloat32, '/mobility/speed_factor', self.set_speed) #NOt working because this is a service in autopilot manager

        # Initialize PID controllers
        self.kp_linear = self.get_parameter_helper('aruco_linear_autopilot_kp', 1.0)
        self.ki_linear = self.get_parameter_helper('aruco_linear_autopilot_ki', 0.0)
        self.kd_linear = self.get_parameter_helper('aruco_linear_autopilot_kd', 0.0)
        Ts_linear = self.get_parameter_helper('aruco_linear_autopilot_Ts', 0.1)
        limit_linear = self.get_parameter_helper('aruco_linear_autopilot_limit', 1.0)

        self.kp_angular = self.get_parameter_helper('aruco_angular_autopilot_kp', 1.0)
        self.ki_angular = self.get_parameter_helper('aruco_angular_autopilot_ki', 0.0)
        self.kd_angular = self.get_parameter_helper('aruco_angular_autopilot_kd', 0.0)
        Ts_angular = self.get_parameter_helper('aruco_angular_autopilot_Ts', 0.1)
        limit_angular = self.get_parameter_helper('aruco_angular_autopilot_limit', 1.0)

        self.low_bound = np.deg2rad(self.get_parameter_helper('low_bound', -45.0))
        self.high_bound = np.deg2rad(self.get_parameter_helper('high_bound', 45.0))

        self.linear_controller = PIDControl(self.kp_linear, self.ki_linear, self.kd_linear, Ts=Ts_linear, limit=limit_linear)
        self.angular_controller = PIDControl(self.kp_angular, self.ki_angular, self.kd_angular, Ts=Ts_angular, limit=limit_angular)

        # State variables
        self.distance = None
        self.angle_to_target = None
        self.enabled = False
        self.rover_vel_cmd = MobilityVelocityCommands()

        self.get_logger().info('Aruco Autopilot Manager is started!')

    def get_parameter_helper(self, name, default_value):
        """Helper function to fetch a parameter or use a default value."""
        return self.declare_parameter(name, default_value).value

    def enable_callback(self, request, response):
        """Service callback to enable or disable the manager."""
        self.enabled = request.data
        response.success = True
        response.message = f"Aruco Autopilot Manager {'enabled' if self.enabled else 'disabled'}."
        return response

    def aruco_autopilot_cmds_callback(self, msg):
        """Callback for autopilot commands."""
        self.distance = msg.distance_to_target
        self.angle_to_target = wrap(msg.angle_to_target, 0)

        # This was used to make it so the rover would either only turn or only drive forward
        # This is not needed anymore because the PID controllers can handle it
        # TODO: We just need to tune the gains for the controller for good performance
        # limit = 8 / 180 * np.pi
        # if abs(self.angle_to_target) > limit:
        #     self.distance = 0

        lin_vel = self.linear_controller.update_with_error(self.distance)
        angular_vel = self.angular_controller.update_with_error(self.angle_to_target)

        self.rover_vel_cmd.u_cmd = lin_vel
        self.rover_vel_cmd.omega_cmd = angular_vel

        self.publish_rover_vel_cmd()

    def set_speed(self, request, response):
        """Service callback to adjust the speed scaling."""
        self.speed = request.data

        if not (0.0 <= self.speed <= 1.0):
            response.success = False
            response.message = "Speed must be between 0 and 1."
            return response

        self.linear_controller.kp = self.speed * self.get_parameter_helper('linear_autopilot_kp', 1.0)
        self.linear_controller.ki = self.speed * self.get_parameter_helper('linear_autopilot_ki', 0.0)
        self.linear_controller.kd = self.speed * self.get_parameter_helper('linear_autopilot_kd', 0.0)

        self.angular_controller.kp = self.speed * self.get_parameter_helper('angular_autopilot_kp', 1.0)
        self.angular_controller.ki = self.speed * self.get_parameter_helper('angular_autopilot_ki', 0.0)
        self.angular_controller.kd = self.speed * self.get_parameter_helper('angular_autopilot_kd', 0.0)

        response.success = True
        response.message = f"Autopilot Speed set to {self.speed}"
        return response

    def publish_rover_vel_cmd(self):
        """Publish velocity commands if enabled."""
        if self.enabled:
            self.rover_vel_cmds_pub.publish(self.rover_vel_cmd)


def main(args=None):
    rclpy.init(args=args)
    aruco_autopilot_manager = ArucoAutopilotManager()

    try:
        rclpy.spin(aruco_autopilot_manager)
    except KeyboardInterrupt:
        pass
    finally:
        aruco_autopilot_manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
