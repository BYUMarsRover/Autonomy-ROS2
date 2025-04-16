#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import glob
import os
import signal
from subprocess import Popen
import cv2 as cv
import numpy as np

from rover_msgs.srv import CameraControl, CameraControlResponse
from rover_msgs.msg import Camera, ScienceFADIntensity

ROS_RATE_HZ = 10


class RoverCameraControl(Node):
    def __init__(self):
        super().__init__('rover_camera_control')

        # Services
        self.create_service(
            CameraControl, 'camera_control', self.camera_control_handler)
        
        # Clients
        self.camera_cleanup = self.create_client(
            CameraControl, 'camera_cleanup')
        
        self.get_logger().info("Initializing camera scripts...")
        self.camera_scripts_init()

        # Publishers
        self.pub_fad_calibration = self.create_publisher(ScienceFADIntensity, '/science_fad_calibration', 1)
        self.get_logger().info("Node initialized")

    def camera_scripts_init(self):
        self.cam_scripts_path = os.path.expanduser(
            '~') + '/BYU-Mars-Rover/scripts/camera/'

        usb_hub_cameras_script_args = ""
        zed_camera_script_args = " -c 1 -d ZED_front -q ZED-left"

        self.single_camera_script = self.cam_scripts_path + "rover-launch-single-camera.sh -c {} -d {} -q Low -a {}"
        # zed_camera_script_args = " -c 1 -d /dev/video0 -q Low"

        self.screenshot_script = self.cam_scripts_path + "rover-take-screenshot.sh -d {} -s {}"

        self.fad_callibration_script = self.cam_scripts_path + "rover-calibrate-fad.sh -d {} -s {}"

        self.usb_hub_cameras_script = self.cam_scripts_path + "rover-launch-usb-hub-cameras.sh" + " " + usb_hub_cameras_script_args
        # self.usb_hub_cameras_script = "ping localhost"
        self.zed_camera_script = self.cam_scripts_path + "rover-launch-single-camera.sh -c {} -d ZED_front -q ZED-left -a {}"
        # self.zed_camera_script = "ping localhost"

        self.usb_hub_camera_process = None
        self.zed_camera_process = None

        self.cameras_init()

        # signal.signal(signal.SIGINT, self.handler_stop_signals)
        # signal.signal(signal.SIGTERM, self.handler_stop_signals)

    def cameras_init(self):
        self.get_logger().info("Initializing cameras...")
        self.usb_hub_cameras = Camera()
        self.usb_hub_cameras.camera_name = "usb_hub_cameras"

        self.zed_camera = Camera()
        self.zed_camera.camera_name = "zed_camera"
        self.zed_camera.quality = "ZED-left"

        self.single_cameras_dict = dict()

    def camera_control_handler(self, req, resp):
        cam = req.camera
        # print("INFO: Handle camera request for \"{}\"".format(cam.camera_name))
        self.get_logger().info("INFO: Handle camera request for \"{}\"".format(cam.camera_name))
        # if req.camera.camera_name == "ZED_front":
        #     if not req.kill:
        #         self.launch_zed_camera(cam.client_address, cam.channel)
        #     else:
        #         self.close_camera(cam.camera_name, self.zed_camera_process)
        # elif req.camera.camera_name == "navView":
        #     if not req.kill:
        #         self.launch_view(cam.client_address, cam.channel, cam.camera_name)
        #     else:
        #         self.close_camera(cam.camera_name, self.single_cameras_dict[cam.camera_name])
        # elif req.camera.camera_name == "fadCam":
        #     if req.calibrate:
        #         intensity = self.calibrate_fad(cam.camera_name, req.site_name)

        #         response = CameraControlResponse()
        #         response.intensity = intensity
        #         response.error = False
        #         response.message = None

        #         print('sending response')
        #         return response
        #     elif req.kill:
        #         self.close_camera(cam.camera_name, self.single_cameras_dict[cam.camera_name])
        #     elif req.screenshot:
        #         self.take_screenshot(cam.client_address, cam.camera_name, req.site_name)
        #     else:
        #         self.launch_single_camera(cam.client_address, cam.channel, cam.camera_name)
        # else:
        print(req.kill, req.screenshot)
        if not req.kill:
            if req.screenshot:
                self.get_logger().info("Taking screenshot\n\n\n!!!\n\n")
                self.take_screenshot(cam.client_address, cam.camera_name, req.site_name)
            else:
                self.launch_single_camera(cam.client_address, cam.channel, cam.camera_name)
        else:
            self.close_camera(cam.camera_name, self.single_cameras_dict[cam.camera_name])
            pass

        resp.error = False  # Set this depending on the logic in your handler
        resp.message = "Camera operation successful"  # Set an appropriate message
        resp.intensity = 0  # Set intensity as needed (e.g., if using FAD calibration)

        return resp
        # return False, None, 0

    def launch_zed_camera(self, base_ip, channel):
        self.zed_camera_process = self.launch_camera(
            "ZED_front", self.zed_camera_process, self.zed_camera_script.format(channel, base_ip))

    def launch_single_camera(self, base_ip, channel, camera_name):
        single_camera_process = None
        single_camera_process = self.launch_camera(
            camera_name, single_camera_process, self.single_camera_script.format(channel, camera_name, base_ip))

        self.single_cameras_dict[camera_name] = single_camera_process

    def launch_view(self, base_ip, channel, camera_name):
        view_process = None
        script = None
        if camera_name == "navView":
            script = self.cam_scripts_path + "rover-launch-navigation-view.sh -a {} -c {}".format(base_ip, channel)
        view_process = self.launch_camera(
            camera_name, view_process, script)

        self.single_cameras_dict[camera_name] = view_process

    def launch_camera(self, camera_name, process, script):
        print("INFO: Launching \"{}\" . . .".format(camera_name))
        if not process or process.poll():
            process = Popen(script, shell=True, preexec_fn=os.setsid)
        else:
            print("ERROR: Camera is already running")
        return process

    def calibrate_fad(self, camera_name, site_name):
        """
            Take a bunch (10+?) of images of the FADD
            get the green upper and lower limit (numpy arrays)
            for each image:
                convert the image to hsv cv.cvtColor(image, cv.COLOR_BGR2HSV)
                mask the image (cv.inRange(hsv, lower_limit, upper_limit))
                bitwise and original image with mask (cv.bitwise_and(image, image, mask=mask))
        """
        print("Calibrating FAD Detector on the rover")
        print(self.fad_callibration_script.format(camera_name, site_name))

        base_path = '/home/marsrover/BYU-Mars-Rover/rover_ws/src/science/src/presentation/resources/'

        #  TODO: USE THIS ONLY IF GLOB DOESN'T WORK, REMOVE THE DATE FROM THE PATH
        # fad_images = [base_path + 'fad_calibration-0.png', base_path + 'fad_calibration-1.png',
        #               base_path + 'fad_calibration-2.png', base_path + 'fad_calibration-3.png',
        #               base_path + 'fad_calibration-4.png', base_path + 'fad_calibration-5.png',
        #               base_path + 'fad_calibration-6.png', base_path + 'fad_calibration-7.png',
        #               base_path + 'fad_calibration-8.png', base_path + 'fad_calibration-9.png',
        #               base_path + 'fad_calibration-10.png']
        #
        # for image in fad_images:
        #     if os.path.exists(image):
        #         os.remove(image)

        print('Starting gstreamer')
        child = Popen(self.fad_callibration_script.format(camera_name, site_name), shell=True, start_new_session=True)
        child.wait(timeout=10)
        print('Gstreamer stopped')

        #  TODO: IF WE GO WITH THIS THEN CHANGE THE SCRIPT TO HAVE A DATE ON THE PATH
        fad_images = glob.glob(base_path + 'fad_calibration*.png')
        print('fad_images', fad_images)

        lower_green = np.array([25, 100, 25])
        upper_green = np.array([100, 255, 255])

        total_intensity = 0
        for image_path in fad_images:
            print('image_path', image_path)
            src = cv.imread(image_path)
            intensity_sum = 0

            hsv_image = cv.cvtColor(src, cv.COLOR_BGR2HSV)
            mask = cv.inRange(hsv_image, lower_green, upper_green)
            result = cv.bitwise_and(hsv_image, hsv_image, mask=mask)

            for row in result:
                for pixel in row:
                    intensity_sum += pixel[2]

            total_intensity += intensity_sum

        intensity = total_intensity

        print('Returning intensity', intensity)
        return intensity

    def take_screenshot(self, address, camera_name, site_name):
        print("Taking screenshot on rover")
        print(self.screenshot_script.format(camera_name, site_name))
        # Open a new process to run the script that takes the screenshot and scp it back to the base station
        child = Popen(self.screenshot_script.format(camera_name, site_name), shell=True, start_new_session=True)
        # Wait for the child process to finish before returning
        child.wait(timeout=10)
        return

    def close_all_cameras(self):
        print("INFO: Closing all cameras . . .")

    def close_camera(self, camera_name, camera_process):
        print("INFO: Closing \"{}\" . . .".format(camera_name))
        if camera_process:
            os.killpg(os.getpgid(
                camera_process.pid), signal.SIGTERM)
        else:
            print("ERROR: Camera is already closed")

    def cleanup(self):
        print("Closing all camera processes . . .")
        self.close_all_cameras()

    def handler_stop_signals(self, signum, frame):
        self.close_all_cameras()
        self.camera_cleanup(cleanup=True)

def main(args=None):
    rclpy.init(args=args)

    rover_camera_control = RoverCameraControl()

    rclpy.spin(rover_camera_control)

    rover_camera_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
