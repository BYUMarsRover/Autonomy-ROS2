import sys
import time

import rclpy
from rclpy.node import Node
from rover_msgs.srv import KeyPress


class KeyboardFSMNode(Node):
    '''
    :author: Nelson Durrant
    :date: November 2024

    ROS2 node that parses through a list of desired keys to press and sends them one by one to the
    KeyPress service.

    Clients:
        - /key_press (rovers_msgs/srv/KeyPress)
    '''

    def __init__(self):
        '''
        Creates a new KeyboardFSM node.
        '''
        super().__init__('keyboard_fsm')
        self.get_logger().info("KeyboardFSMNode started")

        self.cli = self.create_client(KeyPress, '/key_press')
        '''
        Client that calls the KeyPress service to pass in the desired key value.
        '''
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.req = KeyPress.Request()
        '''
        Request object that stores the desired key value.
        '''

    def send_request(self, key):
        '''
        Sends a request to the KeyPress service to press the desired key.
        '''
        self.req.key = ord(key)
        return self.cli.call_async(self.req)


def send_key_press(node, key):
    """
    Calls the KeyPress service and checks to make sure it executed successfully.
    """

    # Keep requesting the service until it is successful
    # Could be replaced with an action and is kind of hacky -- but hey, it works
    completed = False
    while not completed:

        future = node.send_request(key)
        rclpy.spin_until_future_complete(node, future)
        response = future.result()
        if not response:
            node.get_logger().error('Service call failed')
        else:
            if response.success:
                completed = True

        time.sleep(1)  # Throttle the service calls


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardFSMNode()

    time.sleep(5)  # Wait for the other nodes to start up

    # Use this format when calling the launch file:
    # 'ros2 launch keyboard_autonomy keyboard_autonomy_launch.py word:=test'
    keys = list(sys.argv[1])

    for key in keys:
        send_key_press(node, key)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
