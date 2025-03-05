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

from rover_msgs.msg import RoverStateSingleton, MobilityAutopilotCommand, MobilityVelocityCommands, HazardArray, Hazard
from rover_msgs.srv import SetFloat32
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool

# Import wrap from utils and PIDControl from controllers
from mobility.utils.wrap import wrap
from mobility.controllers.pid_control import PIDControl

class AutopilotManager(Node):

    def __init__(self):
        super().__init__('autopilot_manager')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('percent_speed', 0.5),
                ('linear_autopilot_kp', 1.0),
                ('linear_autopilot_ki', 0.0),
                ('linear_autopilot_kd', 0.0),
                ('linear_autopilot_Ts', 0.1),
                ('linear_autopilot_limit', 1.0),
                ('angular_autopilot_kp', 1.0),
                ('angular_autopilot_ki', 0.0),
                ('angular_autopilot_kd', 0.0),
                ('angular_autopilot_Ts', 0.1),
                ('angular_autopilot_limit', 1.0),
                ('low_bound', -45.0),
                ('high_bound', 45.0),
            ]
        )

        self.enabled = False

        # Data to be stored for controller
        self.rover_vel_cmd = MobilityVelocityCommands()
        self.distance = 0
        self.curr_heading = 0
        self.des_heading = 0
        self.course_error = 0

        # Controller gains
        self.speed = self.get_parameter("percent_speed").get_parameter_value().double_value

        self.kp_linear = self.get_parameter("linear_autopilot_kp").get_parameter_value().double_value
        self.ki_linear = self.get_parameter("linear_autopilot_ki").get_parameter_value().double_value
        self.kd_linear = self.get_parameter("linear_autopilot_kd").get_parameter_value().double_value
        Ts_linear = self.get_parameter("linear_autopilot_Ts").get_parameter_value().double_value
        limit_linear = self.get_parameter("linear_autopilot_limit").get_parameter_value().double_value

        self.kp_angular = self.get_parameter("angular_autopilot_kp").get_parameter_value().double_value
        self.ki_angular = self.get_parameter("angular_autopilot_ki").get_parameter_value().double_value
        self.kd_angular = self.get_parameter("angular_autopilot_kd").get_parameter_value().double_value
        Ts_angular = self.get_parameter("angular_autopilot_Ts").get_parameter_value().double_value
        limit_angular = self.get_parameter("angular_autopilot_limit").get_parameter_value().double_value

        self.low_bound = np.deg2rad(self.get_parameter("low_bound").get_parameter_value().double_value)
        self.high_bound = np.deg2rad(self.get_parameter("high_bound").get_parameter_value().double_value)

        # Other variables
        self.heading_plus = 0.0
        self.slow_down = 0.0
        self.too_close_limit = 0.0  # TODO: Choose a reasonable value
        self.detections = []
        self.avoidance_heading = 0.0
        self.heading_alpha = 0.0
        self.obstacle_found = False
        self.scaling_factor = 0.0
        self.critical_distance = 1.0
        self.time_step = 3.0  # Example time step in seconds
        self.last_callback_time = self.get_clock().now()  # Store last callback time
        self.timer = None  # Timer starts as None

        # ROS subscribers
        self.create_subscription(RoverStateSingleton, '/odometry/rover_state_singleton', self.rover_state_singleton_callback, 10)
        self.create_subscription(MobilityAutopilotCommand, '/mobility/autopilot_cmds', self.autopilot_cmds_callback, 10)
        self.create_subscription(HazardArray, '/hazards', self.obstacle_callback, 10)

        # ROS publishers
        self.rover_vel_cmds_pub = self.create_publisher(MobilityVelocityCommands, '/mobility/rover_vel_cmds', 10)

        # ROS services
        self.create_service(SetBool, '/mobility/autopilot_manager/enabled', self.enable)
        self.create_service(SetFloat32, '/mobility/speed_factor', self.set_speed)

        # PID controllers
        self.linear_controller = PIDControl(self.speed * self.kp_linear, self.speed * self.ki_linear, self.speed * self.kd_linear,
                                            Ts=Ts_linear, limit=limit_linear)
        self.angular_controller = PIDControl(self.speed * self.kp_angular, self.speed * self.ki_angular, self.speed * self.kd_angular,
                                             Ts=Ts_angular, limit=limit_angular)

        self.get_logger().info("Autopilot Manager initialized!")


    def autopilot_cmds_callback(self, msg: MobilityAutopilotCommand):
        
        self.distance = msg.distance_to_target

        # self.des_heading = wrap(msg.course_angle + self.heading_plus, 0)
        if self.obstacle_found:
            #figure out new heading to avoid obstacle
            new_heading  = msg.course_angle * (1 - self.heading_alpha) + self.avoidance_heading * self.heading_alpha
            self.des_heading = wrap(new_heading, 0)

        else:
            self.des_heading = wrap(msg.course_angle, 0)

        self.curr_heading = wrap(self.curr_heading, 0)
        self.course_error = wrap(self.des_heading - self.curr_heading, 0)
        # self.get_logger().info(f"Course Error: {self.course_error}")

        lin_vel = self.linear_controller.update_with_error(self.distance)
        angular_vel = self.angular_controller.update_with_error(self.course_error)

        if self.obstacle_found:
            self.rover_vel_cmd.u_cmd = lin_vel * .5
        else:
            self.rover_vel_cmd.u_cmd = lin_vel
        self.rover_vel_cmd.omega_cmd = angular_vel
        self.rover_vel_cmd.course_heading_error = self.course_error

        self.publish_rover_vel_cmd()

    def rover_state_singleton_callback(self, msg: RoverStateSingleton):
        self.curr_heading = np.deg2rad(msg.map_yaw)

    def obstacle_callback(self, msg):
        

        #start a timer to stop avoiding the obstacle after a certain amount of time
        if self.timer is None:
            self.timer = self.create_timer(self.time_step, self.toggle_obstacle_avoidance)
        else:
            self.timer.cancel()  # Cancel the previous timer
            self.timer = self.create_timer(self.time_step, self.toggle_obstacle_avoidance)


        self.obstacle_found = False
        if len(msg.hazards) == 1:

            self.hazard_msg = msg
            self.obstacle_found = True 
            hazard = msg.hazards[0]
            if hazard.type == Hazard.OBSTACLE:

                #Calculate the distance to the obstacle
                dist = np.sqrt(hazard.location_x**2 + hazard.location_y**2)
                max_obs_radius = np.sqrt(hazard.length_x**2 + hazard.length_y**2)*.6 #.6 is a fudge factor
                self.scaling_factor = self.critical_distance + max_obs_radius #the bigger the obstacle, the more space we want to give it

                angle_to_obstacle = np.arctan2(hazard.location_y, hazard.location_x)

                if(hazard.location_y > 0): #obtacle is on the right
                    self.avoidance_heading = angle_to_obstacle - np.pi/2
                    
                elif(hazard.location_y <= 0): #obstacle is on the left
                    self.avoidance_heading = angle_to_obstacle + np.pi/2

                #Calculate the alpha value to to avoid the obstacle
                self.heading_alpha = self.scaling_factor / dist #the closer we are to the obstacle, the more we want to avoid it
                
                #print statements
                self.get_logger().info(f"Hazard Distance: {dist}")
                self.get_logger().info(f"Angle to Obstacle: {angle_to_obstacle}")
                self.get_logger().info(f"Max Obs Radius: {max_obs_radius}")
                self.get_logger().info(f"Scaling Factor: {self.scaling_factor}")
                self.get_logger().info(f"Avoidance Heading: {self.avoidance_heading}")
                self.get_logger().info(f"Alpha: {self.heading_alpha}")
                self.get_logger().info(f"Current Heading: {self.curr_heading}")
                
        #If there are multiple hazards, we need to decide which way we want to try to avoid it
        # else:
        #     for hazard in msg.hazards:
        #         if hazard.hazard_type == Hazard.HAZARD_OBSTACLE:

        #             #Calculate the distance to the obstacle
        #             dist = np.sqrt(hazard.x**2 + hazard.y**2)

        #             #calculate the volume of the obstacle
        #             volume = hazard.length * hazard.width * hazard.height
                    
        #             self.obstacle_avoidance(hazard)

    def toggle_obstacle_avoidance(self):
        #It has been self.time_step seconds since the last obstacle detection, so we can stop avoiding the obstacle
        self.obstacle_found = False
        #turn off the timer
        self.timer.cancel()

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

    def enable(self, request, response):
        self.enabled = request.data
        response.success = True
        response.message = f"Autopilot Manager: {'ENABLED' if self.enabled else 'DISABLED'}"
        return response


def main(args=None):
    rclpy.init(args=args)
    autopilot_manager = AutopilotManager()
    rclpy.spin(autopilot_manager)
    autopilot_manager.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
