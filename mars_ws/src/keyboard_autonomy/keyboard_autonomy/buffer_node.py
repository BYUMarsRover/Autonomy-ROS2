#!/usr/bin/python3

import rospy
from collections import deque

from rover_msgs.srv import DebugMessages, DebugMessagesResponse
from rosgraph_msgs.msg import Log
from rover_msgs.msg import NavStatus

class BufferNode:

    def __init__(self):

        # This subscribes to the rover's autonomy state.
        self.status_sub = rospy.Subscriber('/nav_status', NavStatus, self.status_buffer_callback)

        # This subscribes to the debug messages from all of the network.
        self.log_sub = rospy.Subscriber("/rosout_agg", Log, self.rosout_buffer_callback)

        # Service to send the buffered messages to the GUI.
        self.buffer_srv = rospy.Service('/debug_messages', DebugMessages, self.buffer_service)

        # The buffer of the Status and Debug messages.
        self.buffer : deque[str] = deque(maxlen=100)
        self.current_state : str = ""

    def status_buffer_callback(self, msg : NavStatus):
        # Append the message received to the buffer.
        if msg.state != self.current_state:
            self.current_state = msg.state
            self.buffer.append("State: " + msg.state)

    def rosout_buffer_callback(self, msg : Log):
        # If the message level is debug, append it to the buffer.
        if msg.level == 1:
            self.buffer.append(msg.name + ": " + msg.msg)

    def buffer_service(self, req):
        # Response message to send the buffer.
        response = DebugMessagesResponse()

        msgs_to_send = []

        # Iterate through the buffer and package them to send.
        while self.buffer:
            msgs_to_send.append(self.buffer.popleft())

        # Put the messges in the response.
        response.messages = msgs_to_send

        return response


if __name__ == '__main__':
    """
    Main function for the buffer node
    """
    rospy.init_node('buffer_node')

    buff_node = BufferNode()

    while not rospy.is_shutdown():
        rospy.spin()
        pass