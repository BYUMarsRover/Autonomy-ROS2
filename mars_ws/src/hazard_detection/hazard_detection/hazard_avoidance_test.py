import rclpy
from rclpy.node import Node
from rover_msgs.msg import MobilityVelocityCommands



class HazardAvoidanceTest(Node):
    def __init__(self):
        super().__init__('hazard_avoidance_test')
        

        #Subscriber
        self.vel_cmds_sub = self.create_subscription(
            MobilityVelocityCommands, 
            '/mobility/rover_vel_cmds', 
            self.vel_cmds_callback, 
            10
        )

        


        #Publishers
        self.publisher = self.create_publisher(HazardArray, '/hazards', 10)




def main(args=None):
    rclpy.init(args=args)
    node = HazardAvoidanceTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()