#!/usr/bin/env python3

import rospy
import glob
import os

from rover_msgs.msg import DeviceList

ROS_RATE_HZ = 1


class RoverDeviceUpdater:
    def __init__(self):
        self.dev_publisher = rospy.Publisher(
            '/connected_devices_list', DeviceList, queue_size=1)
        pass

    def update_dev_list(self):
        path = "/dev/rover/"
        device_list_html_dict = {}

        paths = [f for f in glob.glob(path + "**/*", recursive=True)]
        devices = list(dev.split("/dev/rover/")[1]
                       for dev in paths if os.path.islink(dev))
        camera_devices = list(dev.split("/")[-1] for dev in paths if "cameras/" in dev)
        self.dev_publisher.publish(devices=devices, camera_devices=camera_devices)





if __name__ == '__main__':
    rospy.init_node('rover_dev_update')
    rate = rospy.Rate(ROS_RATE_HZ)

    rover_dev_updater = RoverDeviceUpdater()

    while not rospy.is_shutdown():
        rover_dev_updater.update_dev_list()
        rate.sleep()
