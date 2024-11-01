#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import glob
import os

from rover_msgs.msg import DeviceList

ROS_RATE_HZ = 1


class RoverDeviceUpdater(Node):

    def __init__(self):
        super().__init__('rover_dev_update')
        self.dev_publisher = self.create_publisher(
            DeviceList, '/connected_devices_list', 1)
        self.timer = self.create_timer(1.0 / ROS_RATE_HZ, self.update_dev_list)

    def update_dev_list(self):
        path = "/dev/rover/"
        device_list_html_dict = {}

        paths = [f for f in glob.glob(path + "**/*", recursive=True)]
        devices = list(dev.split("/dev/rover/")[1]
                       for dev in paths if os.path.islink(dev))
        camera_devices = list(dev.split("/")[-1] for dev in paths if "cameras/" in dev)
        message = DeviceList()
        message.devices = devices
        message.camera_devices = camera_devices
        self.dev_publisher.publish(message) #TODO "devices" unexpected

def main(args=None):
    rclpy.init(args=args)

    rover_dev_updater = RoverDeviceUpdater()

    rclpy.spin(rover_dev_updater)

    rover_dev_updater.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
