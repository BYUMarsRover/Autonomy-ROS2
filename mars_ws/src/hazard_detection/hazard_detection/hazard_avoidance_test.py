import rclpy
from rclpy.node import Node
from rover_msgs.msg import MobilityAutopilotCommand, MobilityVelocityCommands, HazardArray, Hazard, RoverStateSingleton
from sim.rover_sim import RoverVisualizer as RoverVis
import numpy as np
from numpy import sin, cos, sqrt

class HazardAvoidanceTest(Node):
    def __init__(self):
        super().__init__('hazard_avoidance_test')
        
        #Subscribers
        self.vel_cmds_sub = self.create_subscription(
            MobilityVelocityCommands, 
            '/mobility/rover_vel_cmds', 
            self.vel_cmds_callback, 
            10
        )
        self.autopilot_cmds_sub = self.create_subscription(
            MobilityAutopilotCommand,
            '/mobility/autopilot_cmds',
            self.autopilot_cmds_callback,
            10
        ) 
        
        #Publishers
        self.publisher = self.create_publisher(HazardArray, '/hazards', 10)
        self.autopilot_cmds_pub = self.create_publisher(MobilityAutopilotCommand, '/mobility/autopilot_cmds', 10)
        self.singleton_publisher = self.create_publisher(RoverStateSingleton, '/odometry/rover_state_singleton', 10) 

        #Initialize variables
        self.time_step = 0.1
        self.linear_velocity = 0
        self.angular_velocity = 0
        self.bounding_box_length = 3.0
        self.bounding_box_width = 2.0


    def updateSimulationLoop(self):
        #Update the simulation

        self.vis.set_velocity(self.linear_velocity, self.angular_velocity, self.time_step)
        self.vis.update_display()

        haz_x, haz_y, _, _ = self.vis.get_hazard_locations()
        dist_to_target= self.vis.get_target()
        rov_orientation = self.vis.get_rover_orientation()
        rov_x, rov_y = self.vis.get_rover_position()

        hazard_in_box = self.check_hazard_in_box(haz_x, haz_y, rov_x, rov_y, rov_orientation)
        
        if hazard_in_box:
            #Update hazard according to the simulation and publish
            hazard_msg = HazardArray()
            hazard = Hazard()
            hazard.location_x = haz_x
            hazard.location_y = haz_y
            hazard.location_z = 0
            hazard.length_x = self.haz_length_x
            hazard.length_y = self.haz_length_y
            hazard.length_z = 0
            hazard.type = Hazard.OBSTACLE
            self.hazard_msg.hazards.append(hazard)
            self.publisher.publish(self.hazard_msg)

        #Publish the new dist to target and course angle
        autopilot_msg = MobilityAutopilotCommand()
        autopilot_msg.distance_to_target = dist_to_target
        autopilot_msg.course_angle = self.course_angle
        self.autopilot_cmds_pub.publish(self.autopilot_msg)

        #publish the orientation of the rover
        state_singleton_msg = RoverStateSingleton()
        state_singleton_msg.map_yaw = rov_orientation # TODO: needs to be in degrees
        self.singleton_publisher.publish(state_singleton_msg)

        return

    def check_hazard_in_box(self, haz_x, haz_y, rov_x, rov_y, rov_orientation):
        #Check if hazard is in the bounding box
        hazard_in_box = False
        # Calculate the bounding box coordinates
        half_length = self.bounding_box_length / 2
        half_width = self.bounding_box_width / 2

        # Calculate the corners of the bounding box in the rover's frame
        front_left_x = rov_x + half_length * cos(rov_orientation) - half_width * sin(rov_orientation)
        front_left_y = rov_y + half_length * sin(rov_orientation) + half_width * cos(rov_orientation)
        front_right_x = rov_x + half_length * cos(rov_orientation) + half_width * sin(rov_orientation)
        front_right_y = rov_y + half_length * sin(rov_orientation) - half_width * cos(rov_orientation)
        back_left_x = rov_x - half_length * cos(rov_orientation) - half_width * sin(rov_orientation)
        back_left_y = rov_y - half_length * sin(rov_orientation) + half_width * cos(rov_orientation)
        back_right_x = rov_x - half_length * cos(rov_orientation) + half_width * sin(rov_orientation)
        back_right_y = rov_y - half_length * sin(rov_orientation) - half_width * cos(rov_orientation)

        # Check if the hazard is within the bounding box
        if (min(front_left_x, front_right_x, back_left_x, back_right_x) <= haz_x <= max(front_left_x, front_right_x, back_left_x, back_right_x) and
            min(front_left_y, front_right_y, back_left_y, back_right_y) <= haz_y <= max(front_left_y, front_right_y, back_left_y, back_right_y)):
            hazard_in_box = True

        return hazard_in_box

    def vel_cmds_callback(self, msg):
        self.linear_velocity = msg.u_cmd
        self.angular_velocity = msg.omega_cmd
        return
    
    def autopilot_cmds_callback(self, msg):

        # Create a timer to call `state_loop` every 0.1 seconds (10 Hz)
        self.create_timer(self.time_step, self.updateSimulationLoop)

        #Store the data
        self.distance_to_target = msg.distance_to_target
        self.course_angle = msg.course_angle

        #Initialize the simulation
        self.vis = RoverVis()
        self.vis.set_rover_position(0, 0, 0) #Start with the rover at the origin with orientation of 0
        self.vis.set_target(self.distance_to_target, self.course_angle)
        
        #Set the hazard Location
        self.haz_length_x = 1.5
        self.haz_length_y = 1.5
        self.vis.set_haazards(2, 5, width=self.haz_length_x, height=self.haz_length_y)
    

        return


def main(args=None):
    rclpy.init(args=args)
    node = HazardAvoidanceTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()