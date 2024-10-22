#!/usr/bin/env python3
import rospkg
import rospy
from geometry_msgs.msg import PointStamped
from lat_lon_meter_convertor import LatLonConvertor


class WaypointTranslator:
    def __init__(self):
        rospy.Subscriber("/clicked_point", PointStamped, self.handle_click_callback)
        self.result_publisher = rospy.Publisher(
            "/clicked_lat_lon", PointStamped, queue_size=10
        )
        # rospy.Publisher("/clicked_latlon")
        self.convertor = LatLonConvertor()

    def handle_click_callback(self, msg):
        print(msg.point)
        position = self.convertor.convert_to_latlon(msg.point.x, msg.point.y)

        print(position["lat"])
        print(position["lon"])

        result_msg = PointStamped()

        result_msg.header = msg.header
        result_msg.point.x = position["lon"]
        result_msg.point.y = position["lat"]

        self.result_publisher.publish(result_msg)


if __name__ == "__main__":
    rospy.init_node("waypoint_picker")
    # print("BROSKI WE GOT A CLICKER")
    waypoint = WaypointTranslator()
    rospy.spin()
