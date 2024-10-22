#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header
from nav_msgs.msg import Path as PathMsg
from geometry_msgs.msg import Point, Pose, Quaternion, PoseStamped
from lat_lon_meter_convertor import LatLonConvertor

class PathToMapviz:

    def __init__(self):
        self.path_planning_sub = rospy.Subscriber("/path_planning/smoothed_path", PathMsg, self.path_planning_callback)

        self.pub = rospy.Publisher('/mapviz/path', PathMsg, queue_size=10)

        # Pose array.
        self.poses_array = []

        # Latitude Longitude values to meter coordinate values.
        self.latlonconv = LatLonConvertor()
        self.meters_per_degree_latitude, self.meters_per_degree_longitude = self.latlonconv.get_meters_per_degree_lat_lon()

    def clean(self):
        '''
        This cleans the existing path.
        '''
        self.poses_array.clear()
    
    def path_planning_callback(self, msg):
        '''
        This publishes the path.
        '''
        print("New path received!")
        # ==== Send this path over the publisher ====
        # Make a Quaternion message
        quaternion_msg = Quaternion()
        quaternion_msg.x = 0
        quaternion_msg.y = 0
        quaternion_msg.z = 0
        quaternion_msg.w = 1
        
        # Make a header message
        header_msg = Header()
        header_msg.seq = msg.header.seq
        header_msg.stamp = rospy.Time.now()
        header_msg.frame_id = 'map'

        # Make a path message
        path_msg = PathMsg()
        path_msg.header = header_msg
        

        for i in range(0, len(msg.poses)):
            # Make a Point message
            point_msg = Point()
            point = self.latlonconv.convert_to_meters(msg.poses[i].pose.position.y, msg.poses[i].pose.position.x)
            point_msg.x = point['x']
            point_msg.y = point['y']
            point_msg.z = 1501

            # Make a Pose message
            pose_msg = Pose()
            pose_msg.position = point_msg
            pose_msg.orientation = quaternion_msg

            # Make a Pose stamped message
            pose_stamped_msg = PoseStamped()
            pose_stamped_msg.header = header_msg
            pose_stamped_msg.pose = pose_msg

            # Add to poses array
            self.poses_array.append(pose_stamped_msg)

        path_msg.poses = self.poses_array
        print("Publish It!")
        self.pub.publish(path_msg)

if __name__ == '__main__':
    rospy.init_node('path_to_mapviz')
    transform = PathToMapviz()
    rospy.spin()
