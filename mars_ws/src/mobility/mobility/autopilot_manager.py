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
import time

from rover_msgs.msg import RoverStateSingleton, MobilityAutopilotCommand, MobilityVelocityCommands, ZedObstacles
from rover_msgs.srv import SetFloat32
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool

# Import wrap from utils and PIDControl from controllers
from mobility.utils.wrap import wrap
from mobility.controllers.pid_control import PIDControl

#TODO: implement the manager class
from mobility.utils.manager_interface import Manager


class AutopilotManager(Node):

    def __init__(self):
        super().__init__('autopilot_manager')

        #TODO: clean up this class usage since it was was the parent class in ROS1
        mananger = Manager()

        # Data to be stored for controller
        self.rover_vel_cmd = MobilityVelocityCommands()
        self.distance = 0
        self.curr_heading = 0
        self.des_heading = 0

        self.avoid_hazard_dumb = False
        # Controller gains
        self.speed = self.declare_parameter("percent_speed", 0.5).value

        self.kp_linear = self.declare_parameter("linear_autopilot_kp", 1.0).value
        self.ki_linear = self.declare_parameter("linear_autopilot_ki", 0.0).value
        self.kd_linear = self.declare_parameter("linear_autopilot_kd", 0.0).value
        Ts_linear = self.declare_parameter("linear_autopilot_Ts", 0.1).value
        limit_linear = self.declare_parameter("linear_autopilot_limit", 1.0).value

        self.kp_angular = self.declare_parameter("angular_autopilot_kp", 1.0).value
        self.ki_angular = self.declare_parameter("angular_autopilot_ki", 0.0).value
        self.kd_angular = self.declare_parameter("angular_autopilot_kd", 0.0).value
        Ts_angular = self.declare_parameter("angular_autopilot_Ts", 0.1).value
        limit_angular = self.declare_parameter("angular_autopilot_limit", 1.0).value

        self.low_bound = np.deg2rad(self.declare_parameter("low_bound", -45.0).value)
        self.high_bound = np.deg2rad(self.declare_parameter("high_bound", 45.0).value)

        # Other variables
        self.heading_plus = 0.0
        self.slow_down = 0.0
        self.too_close_limit = 0.0  # TODO: Choose a reasonable value
        self.detections = []

        # ROS subscribers
        self.create_subscription(RoverStateSingleton, '/odometry/rover_state_singleton', self.rover_state_singleton_callback, 10)
        self.create_subscription(MobilityAutopilotCommand, '/mobility/autopilot_cmds', self.autopilot_cmds_callback, 10)
        self.create_subscription(ZedObstacles, '/zed/obstacles', self.obstacle_callback, 10)

        # ROS publishers
        self.rover_vel_cmds_pub = self.create_publisher(MobilityVelocityCommands, '/mobility/rover_vel_cmds', 10)

        # ROS services
        self.create_service(SetBool, '/mobility/autopilot_manager/enabled', mananger.enable)
        self.create_service(SetFloat32, '/mobility/speed_factor', self.set_speed)
        self.create_service(SetBool, '/mobility/hazard_avoidance/enabled', self.enable_hazard_avoidance)
        self.create_service(SetBool, '/mobility/hazard_avoidance/dumb_enabled', self.enable_dumb_hazard_avoidance)

        # PID controllers
        self.linear_controller = PIDControl(self.speed * self.kp_linear, self.speed * self.ki_linear, self.speed * self.kd_linear,
                                            Ts=Ts_linear, limit=limit_linear)
        self.angular_controller = PIDControl(self.speed * self.kp_angular, self.speed * self.ki_angular, self.speed * self.kd_angular,
                                             Ts=Ts_angular, limit=limit_angular)

        self.get_logger().info("Autopilot Manager is started!")

        self.timer = self.create_timer(0.1, self.heading_decay)

    def autopilot_cmds_callback(self, msg: MobilityAutopilotCommand):
        self.distance = msg.distance_to_target

        if not self.avoid_hazard_dumb:
            self.heading_plus = 0.0

        self.des_heading = wrap(msg.course_angle + self.heading_plus, 0)
        self.curr_heading = wrap(self.curr_heading, 0)
        course_error = wrap(self.des_heading - self.curr_heading, 0)

        limit = 8 / 180 * np.pi
        if abs(course_error) > limit and not self.avoid_hazards:
            self.distance = 0

        lin_vel = self.linear_controller.update_with_error(self.distance)
        angular_vel = self.angular_controller.update_with_error(course_error)

        self.rover_vel_cmd.u_cmd = lin_vel
        self.rover_vel_cmd.omega_cmd = angular_vel

        self.publish_rover_vel_cmd()

    def rover_state_singleton_callback(self, msg: RoverStateSingleton):
        self.curr_heading = np.deg2rad(msg.map_yaw)
        self.publish_rover_vel_cmd()

    def enable_hazard_avoidance(self, request: SetBool.Request, response: SetBool.Response):
        self.avoid_hazards = request.data
        response.success = True
        response.message = f"Hazard Avoidance is now {'ON' if self.avoid_hazards else 'OFF'}"
        return response

    def enable_dumb_hazard_avoidance(self, request: SetBool.Request, response: SetBool.Response):
        self.avoid_hazard_dumb = request.data
        response.success = True
        response.message = f"Hazard Avoidance is now {'ON' if self.avoid_hazards else 'OFF'}"
        return response

    def obstacle_callback(self, msg: ZedObstacles):
        if len(msg.x_coord) == 0:
            return

        x_coord = msg.x_coord[0]
        dist = msg.dist[0]
        side = 1 if x_coord > 100 else -1

        self.heading_plus += side
        if abs(dist) < self.too_close_limit:
            self.slow_down += 1

        self.detections.append((time.time_ns(), side))

    def heading_decay(self):
        if not self.detections:
            return
        if time.time_ns() - self.detections[0][0] > 10.0:
            self.heading_plus -= self.detections.pop(0)[1]

    def set_speed(self, request: SetFloat32.Request, response: SetFloat32.Response):
        self.speed = request.data

        self.linear_controller.kp = self.speed * self.kp_linear
        self.linear_controller.ki = self.speed * self.ki_linear
        self.linear_controller.kd = self.speed * self.kd_linear

        self.angular_controller.kp = self.speed * self.kp_angular
        self.angular_controller.ki = self.speed * self.ki_angular
        self.angular_controller.kd = self.speed * self.kd_angular

        response.success = True
        response.message = f"Autopilot Speed set to {self.speed}"
        return response

    def publish_rover_vel_cmd(self):
        if self.rover_vel_cmd:
            self.rover_vel_cmds_pub.publish(self.rover_vel_cmd)


def main(args=None):
    rclpy.init(args=args)
    autopilot_manager = AutopilotManager()
    rclpy.spin(autopilot_manager)
    autopilot_manager.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
