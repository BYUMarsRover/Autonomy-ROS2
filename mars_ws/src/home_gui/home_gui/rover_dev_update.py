#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import glob
import os

from rover_msgs.msg import DeviceList

ROS_RATE_HZ = 1


class RoverDeviceUpdater(Node): # TEMP FIX: CHANGED ALL rover/ TO video* TO CHECK CAMERAS

    def __init__(self):
        super().__init__('rover_dev_update')
        self.dev_publisher = self.create_publisher(
            DeviceList, '/connected_devices_list', 1)
        self.timer = self.create_timer(1.0 / ROS_RATE_HZ, self.update_dev_list)

    def update_dev_list(self):
        # Get all video devices
        paths = glob.glob("/dev/video*")  # List all video devices
        devices = [os.path.basename(dev) for dev in paths]  # Extract filenames
        camera_devices = [dev for dev in devices if dev.startswith("video")]  # Filter cameras
        
        # Create message
        message = DeviceList()
        message.devices = devices  # All detected devices
        message.camera_devices = camera_devices  # Only camera devices

        # Publish message
        self.dev_publisher.publish(message)

        # Debugging logs
        self.get_logger().info(f"Published devices: {devices}")
        self.get_logger().info(f"Published camera devices: {camera_devices}")

        # path = "/dev/video*"
        # device_list_html_dict = {}

        # paths = [f for f in glob.glob(path + "**/*", recursive=True)]
        # devices = list(dev.split(os.path.basename(dev))[1]
        #                for dev in paths if os.path.islink(dev))
        # camera_devices = list(dev.split("/")[-1] for dev in paths if "video" in dev)
        # message = DeviceList()
        # message.devices = devices
        # message.camera_devices = camera_devices
        # self.dev_publisher.publish(message) #TODO "devices" unexpected

def main(args=None):
    rclpy.init(args=args)

    rover_dev_updater = RoverDeviceUpdater()

    rclpy.spin(rover_dev_updater)

    rover_dev_updater.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
