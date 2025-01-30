#!/usr/bin/env python3
"""
Home GUI

This needs to be run with scripts/launch.sh, or else important environment 
variables such as ROS_MASTER_UI and ROS_IP will not be set.
"""

from PyQt5 import uic
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from keyboard_autonomy.keyboard_fsm import KeyboardFSMNode
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray
from rover_msgs.msg import DeviceList, Camera
from rover_msgs.srv import CameraControl
from subprocess import Popen, PIPE
from .html_templates import *
from .dev_name_map import BASE_DEV_NAME_MAP, ROVER_DEV_NAME_MAP
from launch import LaunchService
import os
import launch
import launch.actions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

import sys
import atexit
import os
import signal
import glob
import time
import threading
from datetime import datetime

DEV_UPDATE_PERIOD = 2  # seconds
NUM_OF_CHANNEL = 5


class DeviceListUpdater(QRunnable):
    def __init__(self, trigger):
        # Call the inherited classes __init__ method
        super(DeviceListUpdater, self).__init__()
        self.trigger = trigger

    @pyqtSlot()
    def run(self):
        while (True):
            self.trigger()
            time.sleep(DEV_UPDATE_PERIOD)


class HomeGuiUI(Node, QWidget):
    # rover_dev_list_trigger = pyqtSignal()
    base_dev_list_trigger = pyqtSignal()

    def __init__(self):
        # Call the inherited classes __init__ method
        Node.__init__(self, 'base_home_gui')
        QWidget.__init__(self)
        # Load the .ui file
        uic.loadUi(
            os.path.expanduser('~') + '/mars_ws/src/home_gui/home_gui.ui', self)
        self.show()  # Show the GUI

        self.device_list_QLabel = self.roverDeviceList
        self.rover_last_update_QLabel = self.roverLastUpdate
        self.base_last_update_QLabel = self.baseLastUpdate

        self.threadpool = QThreadPool()
        print("Multithreading with maximum %d threads" %
              self.threadpool.maxThreadCount())

        base_devlist_updater = DeviceListUpdater(
            self.emit_base_dev_list_trigger)

        self.base_dev_list_trigger.connect(self.update_base_dev_list)

        self.threadpool.start(base_devlist_updater)

        self.dev_subscriber = self.create_subscription(
            DeviceList, '/connected_devices_list', self.update_rover_dev_list, 1)

        self.ir_subscriber = self.create_subscription(UInt16MultiArray, '/IR', self.update_ir_distances, 1)
        self.last_left_ir = None
        self.last_right_ir = None
        self.ir_alpha = 0.95
        self.gripper_length = 14

        self.cameraCloseAllButton.clicked.connect(self.close_all_cameras)

        self.cameraLaunchSingleButton.clicked.connect(
            self.launch_single_camera)
        self.cameraCloseSingleButton.clicked.connect(self.close_single_camera)

        self.cameraScreenshotButton.clicked.connect(
            self.take_screenshot)

        self.autonomousKeyboardButton.clicked.connect(
            self.launch_autonomous_keyboard
        )

        self.cancelAutonomousKeyboardButton.clicked.connect(
            self.cancelAutonomousKeyboard
        )

        self.cameraLaunchViewButton.clicked.connect(self.launch_view)
        # self.clickerButton.clicked.connect(self.activate_clicker)

        self.brightnessSlider.valueChanged.connect(self.update_brightness)
        self.contrastSlider.valueChanged.connect(self.update_contrast)

        self.brightness = 100
        self.contrast = 100
        self.rover_camera_list = list()

        self.base_ip = self.get_base_ip()
        print("**********************", self.base_ip)
        self.cameras_init()
        self.channels_init()
        self.camera_scripts_init()

        self.single_camera_processes = list()
        self.single_camera_dict = dict()

        atexit.register(self.cleanup)

        self.camera_control = self.create_client(
            CameraControl, 'camera_control')
        self.create_service(
            CameraControl, 'camera_cleanup', self.handle_camera_cleanup)
        self.keyboard_process = None
        signal.signal(signal.SIGINT, self.handler_stop_signals)
        signal.signal(signal.SIGTERM, self.handler_stop_signals)
        self.script_path = "~/Autonomy-ROS2\mars_ws\src\keyboard_autonomy\scripts\keyboard_autonomy.sh"

    def error(self, message, title='Error'):
        self.get_logger().error(message)

    def popup(self, title, message):
        QMessageBox.about(self, title, message)

    def get_base_ip(self):
        ip = os.getenv("BASE_ADDRESS")
        if ip is None:
            ip = "192.168.1.65"
        return ip

    def cameras_init(self):
        self.single_cameras = list()
        self.busy_camera_list = list()

    def channels_init(self):
        self.channel_dict = dict()

        for i in range(NUM_OF_CHANNEL):
            self.channel_dict[str(i)] = False

    def camera_scripts_init(self):
        cam_scripts_path = os.path.expanduser(
            '~') + '/BYU-Mars-Rover/scripts/camera/'

        self.launch_camera_script = cam_scripts_path + "base-launch-camera-window.sh -c {} -b {} -o {}"

    def get_available_channel(self):
        for c, is_occupied in self.channel_dict.items():
            if not is_occupied:
                return c
        return None

    def free_channel(self, channel):
        self.channel_dict[channel] = False
        return

    def is_camera_busy(self, camera_name):
        for busy_camera in self.busy_camera_list:
            if camera_name == busy_camera:
                return True
        return False

    def update_brightness(self):
        self.brightness = self.brightnessSlider.value()
        self.brightnessLabel.setText("{}%".format(self.brightness))

    def update_contrast(self):
        self.contrast = self.contrastSlider.value()
        self.contrastLabel.setText("{}%".format(self.contrast))

    def launch_single_camera(self):
        channel = self.get_available_channel()
        if channel is None:
            print("ERROR: No available channels")
            return

        camera_name = self.cameraSelector.currentText()
        if self.is_camera_busy(camera_name):
            print("ERROR: Camera \"{}\" is busy".format(camera_name))
            return

        single_camera = Camera()
        single_camera.client_address = self.base_ip
        single_camera.camera_name = camera_name
        single_camera.channel = channel

        self.single_cameras.append(
            single_camera
        )

        single_camera_process = None
        single_camera_process = self.launch_camera(
            single_camera, single_camera_process, None, 
        )

        if single_camera_process:
            self.single_camera_dict[camera_name] = (single_camera, single_camera_process)
        return

    def launch_camera(self, camera, process, script):
        print("INFO: Launching \"{}\" . . .".format(camera.camera_name))
        try:
            self.rover_launch_camera(camera)
        except Exception as e: #TODO
            print(e)
            print(
                "HINT: Rover camera control node did not respond. Is the rover connected?")
            return process
        return self.base_launch_camera(camera, process, script)

    def base_launch_camera(self, camera, process, script):
        script = self.launch_camera_script.format(camera.channel, self.brightness, self.contrast)
        if not process or process.poll():
            process = Popen(script, shell=True, preexec_fn=os.setsid,
                            stderr=PIPE)
            self.start_cam_proc_listener_thread(
                self.close_camera, camera, process)
            print("INFO: Base camera process for \"{}\" launched!".format(camera.camera_name))
        else:
            print("ERROR: Base camera process is already running")
        self.channel_dict[camera.channel] = True
        self.busy_camera_list.append(camera.camera_name)
        return process

    def start_cam_proc_listener_thread(self, close_camera, camera, camera_proc):
        """
        Runs the given args in a subprocess.Popen, and then calls the function
        on_exit when the subprocess completes.
        on_exit is a callable object, and popen_args is a list/tuple of args that 
        would give to subprocess.Popen.
        """
        # returns immediately after the thread starts
        def run_in_thread(close_camera, camera, camera_proc):
            error = camera_proc.stderr.readline()
            if error:
                close_camera(camera, camera_proc)
                print("ERROR: Camera run error:", error)
            return
        thread = threading.Thread(target=run_in_thread,
                                  args=(close_camera, camera, camera_proc))
        thread.start()
        # returns immediately after the thread starts
        return thread

    def rover_launch_camera(self, camera):
        print("INFO: Signal rover to launch \"{}\" . . .".format(camera.camera_name))
        self.camera_control(camera=camera)

    def take_screenshot(self):
        camera_name = self.cameraSelector.currentText()
        site_name = self.screenshotLineEdit.text()

        if not site_name:
            self.popup("Screenshot Error", "You need to input text for the screenshot filename")
            return

        relaunch_camera = camera_name in self.single_camera_dict

        if relaunch_camera:
            self.close_single_camera()

        try:
            self.rover_take_screenshot(camera_name, site_name)
        # This could be an OSError or a ValueError
        except Exception as error:
            print(str(error))
        finally:
            if relaunch_camera:
                self.launch_single_camera()
            else:
                print("not relaunching camera")

    def rover_take_screenshot(self, camera_name, site_name):
        print("INFO: Signal rover to take screenshot on \"{}\" . . .".format(camera_name))
        camera = Camera()
        camera.client_address = "{}@{}".format(os.getlogin(), self.base_ip)
        camera.camera_name = camera_name
        # In ROS 1 services are synchronous so this will block until the service finishes
        self.camera_control(camera=camera, site_name=site_name, screenshot=True)

    def launch_autonomous_keyboard(self):
        word = self.autonomousKeyboardLineEdit.text()
        print(word)
        self.get_logger().info(word)
        if self.keyboard_process is None or self.keyboard_process.poll() is not None:
            self.get_logger().info("starting keyboard autonomy launch file")
            self.keyboard_process = Popen(self.script_path, shell=True, preexec_fn=os.setsid, stderr=PIPE)
            self.get_logger().info("launched the keyboard autonomy file")
        else:
            self.get_logger().info('Launch file already running.')

        
        # rclpy.init(args=word)
        # node = KeyboardFSMNode()
        # rclpy.spin(node)

    def cancelAutonomousKeyboard(self):
        reply = QMessageBox.question(self, 'Confirm Cance', 'Are yu sure you want to cancel?', QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if reply == QMessageBox.Yes:
            self.keyboard_process.terminate()
            try:
                self.keyboard_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.get_logger().info("Graceful termination failed, killing process...")
                self.keyboard_process.kill()
            self.get_logger().info("Process terminated.")
            # print("Keboard typing killed successfully")



    def launch_view(self):
        channel = self.get_available_channel()
        if channel is None:
            print("ERROR: No available channels")
            return

        camera_name = self.cameraViewSelector.currentText()
        if self.is_camera_busy(camera_name):
            print("ERROR: Camera \"{}\" is busy".format(camera_name))
            return

        single_camera = Camera()
        single_camera.client_address = self.base_ip
        single_camera.camera_name = camera_name
        single_camera.channel = channel

        self.single_cameras.append(
            single_camera
        )

        single_camera_process = None
        single_camera_process = self.launch_camera(
            single_camera, single_camera_process, None,
        )

        if single_camera_process:
            self.single_camera_dict[camera_name] = (single_camera, single_camera_process)
        return

    # def activate_clicker(self):
    #     print("INFO: clicker button clicked")

    def close_all_cameras(self):
        print("INFO: Closing all cameras . . .")
        self.close_single_cameras()

    def close_single_cameras(self):
        for camera_name, camera_tuple in self.single_camera_dict.items():
            self.close_camera(camera_tuple[0], camera_tuple[1])

    def close_single_camera(self):
        camera_name = self.cameraSelector.currentText()
        try:
            cam_proc_tuple = self.single_camera_dict[camera_name]
        except KeyError:
            print("ERROR: Camera already not running")
            return
        self.close_camera(cam_proc_tuple[0], cam_proc_tuple[1])

    def close_camera(self, camera, camera_process):
        print("INFO: Closing \"{}\" . . .".format(camera.camera_name))
        try:
            self.rover_close_camera(camera)
        except Exception as e:
            print(e)
            print(
                "HINT: Rover camera control node did not respond. Is the rover connected?")
        self.base_close_camera(camera, camera_process)

    def base_close_camera(self, camera, camera_process):
        try:
            self.busy_camera_list.remove(camera.camera_name)
        except ValueError:
            pass
        try:
            self.channel_dict[camera.channel] = False
        except:
            pass
        if camera_process:
            os.killpg(os.getpgid(
                camera_process.pid), signal.SIGTERM)
            print("INFO: Base process for \"{}\" closed!".format(camera.camera_name))
        else:
            print("ERROR: Base camera process already closed")

    def rover_close_camera(self, camera):
        print("INFO: Signal rover to close \"{}\" . . .".format(camera.camera_name))
        self.camera_control(camera=camera, kill=True)

    def emit_rover_dev_list_trigger(self):
        self.rover_dev_list_trigger.emit()

    def emit_base_dev_list_trigger(self):
        self.base_dev_list_trigger.emit()

    def update_base_dev_list(self):
        path = "/dev/rover/"

        paths = [f for f in glob.glob(path + "**/*", recursive=True)]
        devices = list(dev for dev in paths if os.path.islink(dev))
        html = ""

        for d_path, d_name in BASE_DEV_NAME_MAP.items():
            if "/dev/rover/" + d_path in devices:
                is_connected = True
            else:
                is_connected = False
            html += generate_status_html(d_name, is_connected)

        self.baseDeviceList.setText(html)
        self.refresh_update_label(self.baseLastUpdate)

    def update_rover_dev_list(self, ros_message):
        devices = ros_message.devices
        camera_devices = ros_message.camera_devices
        html = ""

        for d_path, d_name in ROVER_DEV_NAME_MAP.items():
            if d_path in devices:
                is_connected = True
            else:
                is_connected = False
            html += generate_status_html(d_name, is_connected)

        self.update_rover_camera_list(camera_devices)
        self.roverDeviceList.setText(html)
        self.refresh_update_label(self.roverLastUpdate)

    def update_ir_distances(self, ir_readings):
        left_ir_distance = 17.8 * (ir_readings.data[0] / 1023.0 * 5) ** -0.872 - self.gripper_length
        right_ir_distance = 17.8 * (ir_readings.data[1] / 1023.0 * 5) ** -0.872 - self.gripper_length

        if (self.last_left_ir is None):
            self.last_left_ir = left_ir_distance
        else:
            self.last_left_ir = left_ir_distance * (1 - self.ir_alpha) + self.last_left_ir * self.ir_alpha
        if (self.last_right_ir is None):
            self.last_right_ir = right_ir_distance
        else:
            self.last_right_ir = right_ir_distance * (1 - self.ir_alpha) + self.last_right_ir * self.ir_alpha

        if (round(self.last_left_ir) > 0 and round(self.last_left_ir) < 20):
            self.left_ir_label.setText("Left IR: " + str(round(self.last_left_ir)) + "cm")
        else:
            self.left_ir_label.setText("Left IR: NA")
        if (round(self.last_right_ir) > 0 and round(self.last_right_ir) < 20):
            self.right_ir_label.setText("Right IR: " + str(round(self.last_right_ir)) + "cm")
        else:
            self.right_ir_label.setText("Right IR: NA")

    def update_rover_camera_list(self, camera_devices):
        selected_device = self.cameraSelector.currentText()
        self.cameraSelector.clear()
        for d in camera_devices:
            self.cameraSelector.addItem(d)
        self.cameraSelector.setCurrentText(selected_device)

    def refresh_update_label(self, label):
        time_str = datetime.now().strftime("%d/%m/%Y %H:%M:%S")
        label.setText(time_str)

    def cleanup(self):
        print("Closing all camera processes . . .")
        self.close_all_cameras()

    def handle_camera_cleanup(self, req):
        if req.cleanup:
            self.cleanup()
        return False, None

    def handler_stop_signals(self, signum, frame):
        print("Closing all camera processes . . .")
        self.close_all_cameras()

def main(args=None):
    rclpy.init(args=args)
    Popen("pkill gst", shell=True, preexec_fn=os.setsid, stderr=PIPE)
    app = QApplication(sys.argv)
    window = HomeGuiUI()
    # Start GUI app
    app.exec_()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
