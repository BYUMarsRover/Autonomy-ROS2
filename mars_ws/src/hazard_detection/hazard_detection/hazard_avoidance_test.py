import rclpy
from rclpy.node import Node
from rover_msgs.msg import MobilityAutopilotCommand, MobilityVelocityCommands, HazardArray, Hazard, RoverStateSingleton
from hazard_detection.sim.rover_sim import RoverVisualizer as RoverVis
import numpy as np
from numpy import sin, cos, sqrt
from std_srvs.srv import SetBool

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

        #Service Clients
        self.autopilot_manager_client = self.create_client(SetBool, '/mobility/autopilot_manager/enabled')

        #Initialize variables
        self.time_step = 0.1
        self.linear_velocity = 0
        self.angular_velocity = 0
        self.bounding_box_height = 3.0
        self.bounding_box_width = 2.0
        self.vis = None
        self.rover_height = 0.0
        self.rover_width = 0.0

        self.get_logger().info('Hazard Avoidance Test Node Initialized')


    def updateSimulationLoop(self):
        #Update the simulation

        self.vis.set_velocity(self.linear_velocity, self.angular_velocity, self.time_step)
        self.vis.update_display()

        haz_x, haz_y, haz_width, haz_height = self.vis.get_hazard_locations() #In the rover frame
        dist_to_target, course_angle = self.vis.get_target()
        rov_orientation = self.vis.get_rover_orientation()

        haz_max_radius = sqrt((haz_width/2)**2 + (haz_height/2)**2) / 2
        hazard_in_box = self.check_hazard_in_box(haz_x, haz_y, haz_max_radius)
        
        if hazard_in_box:

            self.get_logger().info(f'Hazard in bounding box. H_x: {haz_x}, H_y: {haz_y}')

            hazard_msg = HazardArray()
            hazard = Hazard()
            hazard.location_x = haz_x
            hazard.location_y = haz_y
            hazard.location_z = 0.0
            hazard.length_x = self.haz_length_x
            hazard.length_y = self.haz_length_y
            hazard.length_z = 0.0
            hazard.type = Hazard.OBSTACLE
            hazard_msg.hazards.append(hazard)
            hazard_msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(hazard_msg)

        #Publish the new dist to target and course angle
        autopilot_msg = MobilityAutopilotCommand()
        autopilot_msg.distance_to_target = dist_to_target
        autopilot_msg.course_angle = course_angle
        self.autopilot_cmds_pub.publish(autopilot_msg)

        #publish the orientation of the rover
        state_singleton_msg = RoverStateSingleton()
        state_singleton_msg.map_yaw = np.rad2deg(rov_orientation) 
        self.singleton_publisher.publish(state_singleton_msg)

        return

    def check_hazard_in_box(self, haz_x, haz_y, haz_max_radius):
        #Check if hazard is in the bounding box
        hazard_in_box = False
        half_width = self.rover_width / 2
        half_height = self.rover_height / 2

        # Calculate the corners of the bounding box (rectangle in front of the rover)
        #X is forward, Y is to the right
        
        back_left_y = -self.bounding_box_width/2
        back_left_x = half_height 
        back_right_y = self.bounding_box_width/2
        back_right_x = half_height
        front_left_y =  -self.bounding_box_width/2
        front_left_x = half_height + self.bounding_box_height
        front_right_y = self.bounding_box_width/2
        front_right_x = half_height + self.bounding_box_height

        # Check if the hazard is within the bounding box (check all corners)
        if (min(front_left_x, front_right_x, back_left_x, back_right_x) <= haz_x <= max(front_left_x, front_right_x, back_left_x, back_right_x) and
            min(front_left_y, front_right_y, back_left_y, back_right_y) <= haz_y <= max(front_left_y, front_right_y, back_left_y, back_right_y)):
            hazard_in_box = True
        elif (min(front_left_x, front_right_x, back_left_x, back_right_x) <= haz_x + haz_max_radius <= max(front_left_x, front_right_x, back_left_x, back_right_x) and
            min(front_left_y, front_right_y, back_left_y, back_right_y) <= haz_y + haz_max_radius <= max(front_left_y, front_right_y, back_left_y, back_right_y)):
            hazard_in_box = True
        elif (min(front_left_x, front_right_x, back_left_x, back_right_x) <= haz_x - haz_max_radius <= max(front_left_x, front_right_x, back_left_x, back_right_x) and
            min(front_left_y, front_right_y, back_left_y, back_right_y) <= haz_y - haz_max_radius <= max(front_left_y, front_right_y, back_left_y, back_right_y)):
            hazard_in_box = True
        elif (min(front_left_x, front_right_x, back_left_x, back_right_x) <= haz_x + haz_max_radius <= max(front_left_x, front_right_x, back_left_x, back_right_x) and
            min(front_left_y, front_right_y, back_left_y, back_right_y) <= haz_y - haz_max_radius <= max(front_left_y, front_right_y, back_left_y, back_right_y)):
            hazard_in_box = True
        elif (min(front_left_x, front_right_x, back_left_x, back_right_x) <= haz_x - haz_max_radius <= max(front_left_x, front_right_x, back_left_x, back_right_x) and
            min(front_left_y, front_right_y, back_left_y, back_right_y) <= haz_y + haz_max_radius <= max(front_left_y, front_right_y, back_left_y, back_right_y)):
            hazard_in_box = True
        

        return hazard_in_box

    def vel_cmds_callback(self, msg):
        self.linear_velocity = msg.u_cmd
        self.angular_velocity = msg.omega_cmd
        return
    
    def autopilot_cmds_callback(self, msg):
        #As soon as we recieve one autopilot cmd via terminal, we will start the simulation

        if self.vis is None: #only do it on the first message
            #Enable the autopilot manager
            self._toggle_enable_autopilot_manager(True)

            # Create a timer to call `state_loop` every 0.1 seconds (10 Hz)
            self.create_timer(self.time_step, self.updateSimulationLoop)

            #Store the data
            distance_to_target = msg.distance_to_target
            course_angle = msg.course_angle

            #Initialize the simulation
            self.vis = RoverVis()
            self.vis.set_rover_position(0, 0, 0) #Start with the rover at the origin with orientation of 0
            
            target_x = distance_to_target * sin(course_angle)
            target_y = distance_to_target * cos(course_angle)
            self.vis.set_target(target_x, target_y)
            
            #Set the hazard Location
            self.haz_length_x = 1.0
            self.haz_length_y = 1.0
            self.vis.add_hazard(2, 5, width=self.haz_length_x, height=self.haz_length_y)

            self.rover_width, self.rover_height = self.vis.get_rover_dims()
            self.get_logger().info('Simulation Initialized')

        return
    

    def _toggle_enable_autopilot_manager(self, enable: bool):
        client = self.autopilot_manager_client
        while not client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Waiting for service /mobility/autopilot_manager/enabled...')
        request = SetBool.Request()
        request.data = enable
        future = client.call_async(request)



def main(args=None):
    rclpy.init(args=args)
    node = HazardAvoidanceTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()