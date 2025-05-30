# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String
# from rover_msgs.msg import DeviceList, Camera
# from rover_msgs.srv import CameraControl
# from subprocess import Popen, PIPE
# from .html_templates import *
# from .dev_name_map import BASE_DEV_NAME_MAP, ROVER_DEV_NAME_MAP
# import asyncio

# import sys
# import atexit
# import os
# import signal
# import glob
# import time
# import threading
# from datetime import datetime

# DEV_UPDATE_PERIOD = 2  # seconds
# NUM_OF_CHANNEL = 5

# class CameraHandler(Node):

#     def __init__(self):
#         Node.__init__(self, 'camera_handler')

#         #Subscribe to the base home gui
#         self.single_camera_subscriber = self.create_subscription(String, '/launch_single_camera', self.launch_single_camera, 1)
#         self.close_single_camera_subscriber = self.create_subscription(String, '/close_single_camera', self.close_single_camera, 1)

#         #Publish to the base home gui

#         self.brightness = 100
#         self.contrast = 100
#         self.rover_camera_list = list()

#         self.base_ip = self.get_base_ip()
#         print("**********************", self.base_ip)
#         self.cameras_init()
#         self.channels_init()
#         self.camera_scripts_init()

#         self.single_camera_processes = list()
#         self.single_camera_dict = dict()

#         # atexit.register(self.cleanup)

#         self.camera_control = self.create_client(
#             CameraControl, 'camera_control')
#         self.create_service(
#             CameraControl, 'camera_cleanup', self.handle_camera_cleanup)

#         signal.signal(signal.SIGINT, self.handler_stop_signals)
#         signal.signal(signal.SIGTERM, self.handler_stop_signals)

#     def get_base_ip(self):
#         ip = os.getenv("BASE_ADDRESS")launch
#         if ip is None:
#             ip = "192.168.1.65"
#         return ip
    
#     def cameras_init(self):
#         self.single_cameras = list()
#         self.busy_camera_list = list()

#     def channels_init(self):
#         self.channel_dict = dict()

#         for i in range(NUM_OF_CHANNEL):
#             self.channel_dict[str(i)] = False

#     def camera_scripts_init(self):
#         cam_scripts_path = os.path.expanduser(
#             '~') + '/BYU-Mars-Rover/scripts/camera/'

#         self.launch_camera_script = cam_scripts_path + "base-launch-camera-window.sh -c {} -b {} -o {}"

#     def get_available_channel(self):
#         for c, is_occupied in self.channel_dict.items():
#             if not is_occupied:
#                 return c
#         return None

#     def free_channel(self, channel):
#         self.channel_dict[channel] = False
#         return

#     def is_camera_busy(self, camera_name):
#         for busy_camera in self.busy_camera_list:
#             if camera_name == busy_camera:
#                 return True
#         return False

#     def update_brightness(self):
#         self.brightness = self.brightnessSlider.value()
#         self.brightnessLabel.setText("{}%".format(self.brightness))

#     def update_contrast(self):
#         self.contrast = self.contrastSlider.value()
#         self.contrastLabel.setText("{}%".format(self.contrast))

#     def launch_single_camera(self, msg):
#         # camera_config = CameraConfig(name="microscopeCam", param_path='mars_ws/src/start/config/cam_config/params_1.yaml')

#         channel = self.get_available_channel()
#         self.get_logger().info(f"Got to launch_single_camera on channel {channel}")
#         if channel is None:
#             print("ERROR: No available channels")
#             return

#         camera_name = msg.data
#         if self.is_camera_busy(camera_name):
#             print("ERROR: Camera \"{}\" is busy".format(camera_name))
#             return

#         single_camera = Camera()
#         # single_camera.client_address = self.base_ip
#         single_camera.client_address = "10.0.0.4"
#         single_camera.camera_name = camera_name
#         single_camera.channel = channel

#         self.single_cameras.append(
#             single_camera
#         )

#         single_camera_process = None
#         print("launching camera into launch_camera")
#         single_camera_process = self.launch_camera(
#             single_camera, single_camera_process, None, 
#         )

#         if single_camera_process:
#             self.single_camera_dict[camera_name] = (single_camera, single_camera_process)
#         return

#     def launch_camera(self, camera, process, script):
#         self.get_logger().info("INFO: Launching \"{}\" . . .".format(camera.camera_name))
#         try:
#             self.rover_launch_camera(camera)
#         except Exception as e:
#             self.get_logger().info("HINT: Rover camera control node did not respond. Is the rover connected?")
#             return process
        
#         print("Getting to base_launch")
#         # Call the base_launch_camera asynchronously to prevent blocking
#         return self.base_launch_camera(camera, process, script)

#     async def base_launch_camera(self, camera, process, script):  # Make this method async
#         script = self.launch_camera_script.format(camera.channel, self.brightness, self.contrast)
        
#         if not process or process.poll():
#             self.get_logger().info(f"Trying to launch script: {script}")

#             # Ensure to await the process asynchronously
#             process = await self.start_camera_process(script)  # Now we can await the process directly
#             if process:
#                 self.get_logger().info(f"INFO: Base camera process for \"{camera.camera_name}\" launched!")
#             else:
#                 self.get_logger().error("ERROR: Failed to launch camera process")
#         else:
#             self.get_logger().error(f"ERROR: Base camera process is already running for \"{camera.camera_name}\"")

#         self.channel_dict[camera.channel] = True
#         self.busy_camera_list.append(camera.camera_name)
#         return process

    
#     def base_close_camera(self, camera, camera_process):
#         try:
#             # Wait for the process to finish if it's a coroutine
#             if asyncio.iscoroutine(camera_process):
#                 camera_process = asyncio.run(camera_process)

#             # Now camera_process should be a Popen object
#             self.busy_camera_list.remove(camera.camera_name)
#         except ValueError:
#             pass

#         try:
#             self.channel_dict[camera.channel] = False
#         except:
#             pass

#         if camera_process:
#             try:
#                 os.killpg(os.getpgid(camera_process.pid), signal.SIGTERM)
#                 self.get_logger().info(f"INFO: Base process for \"{camera.camera_name}\" closed!")
#             except Exception as e:
#                 self.get_logger().error(f"Failed to kill process: {e}")
#         else:
#             self.get_logger().error("ERROR: Base camera process already closed")
    
#     async def start_camera_process(self, script):
#         try:
#             self.get_logger().info(f"Attempting to start camera process with command: {script}")

#             # Start the process asynchronously using asyncio.create_subprocess_shell
#             process = await asyncio.create_subprocess_shell(
#                 script,
#                 stdout=PIPE,
#                 stderr=PIPE,
#                 preexec_fn=os.setsid  # Ensures process is run in its own session
#             )

#             # Handle both stdout and stderr streams asynchronously
#             async def read_output(process):
#                 stdout, stderr = await process.communicate()
#                 if stdout:
#                     self.get_logger().info(f"Camera Process Output: {stdout.decode()}")
#                 if stderr:
#                     self.get_logger().error(f"Camera Process Error: {stderr.decode()}")

#             # Start reading the output
#             asyncio.create_task(read_output(process))

#             # Wait for the process to finish and return the process object
#             await process.wait()  # Waits for the process to finish
#             return process

#         except Exception as e:
#             self.get_logger().error(f"Failed to start camera process: {e}")
#             return None




#     def start_cam_proc_listener_thread(self, close_camera, camera, camera_proc):
#         """
#         Runs the given args in a subprocess.Popen, and then calls the function
#         on_exit when the subprocess completes.
#         on_exit is a callable object, and popen_args is a list/tuple of args that 
#         would give to subprocess.Popen.
#         """
#         # returns immediately after the thread starts
#         def run_in_thread(close_camera, camera, camera_proc):
#             error = camera_proc.stderr.readline()
#             if error:
#                 close_camera(camera, camera_proc)
#                 print("ERROR: Camera run error:", error)
#             return
#         thread = threading.Thread(target=run_in_thread,
#                                   args=(close_camera, camera, camera_proc))
#         thread.start()
#         # returns immediately after the thread starts
#         return thread

#     def rover_launch_camera(self, camera):

#         print(camera)
#         # print("INFO: Signal rover to launch \"{}\" . . .".format(camera.camera_name))
#         # # self.camera_control(camera=camera) #Next line is an attempt to call the service with already passed stuff for debugging.
#         # future = self.camera_control.call_async(camera=camera, sitename="site-1", kill=False, screenshot=True, cleanup=False, calibrate=False)
#         # future.add_done_callback(self.camera_control_response_callback)
#         # print("Should be done...")
#         # Create a request object
#         req = CameraControl.Request()
#         print("Created camera control object")
        
#         # Call the service asynchronously
#         req.camera = camera
#         print("Set camera attribute")
#         req.site_name = "site-1"
#         print("Set sitename attribute")
#         req.kill = False
#         print("Set kill attribute")
#         req.screenshot = False
#         print("Set screenshot attribute")
#         req.cleanup = False
#         print("Set cleanup attribute")
#         req.calibrate = False            
#         print("finish camera control object")
#         print("Set calibrate attribute")

#         try:
#             future = self.camera_control.call_async(req)
#             future.add_done_callback(self.camera_control_response_callback)  # Add a callback to handle the response
#             self.get_logger().info("Service call made, waiting for response...")

#         except Exception as e:
#             print("Error calling the service")
#             self.get_logger().error(f"Service call failed: {e}")

#     def camera_control_response_callback(self, future):
#         try:
#             response = future.result()  # Get the result of the service call
#             if response:
#                 self.get_logger().info(f"Camera control response: {response}")
#             else:
#                 self.get_logger().error("Camera control service did not respond properly.")
#         except Exception as e:
#             self.get_logger().error(f"Error in service response: {e}")

#     def close_all_cameras(self):
#         print("INFO: Closing all cameras . . .")
#         self.close_single_cameras()

#     def close_single_cameras(self):
#         for camera_name, camera_tuple in self.single_camera_dict.items():
#             self.close_camera(camera_tuple[0], camera_tuple[1])

#     def close_single_camera(self, msg):
#         self.get_logger().info("Got to close_single_camera")
#         camera_name = msg.data
#         try:
#             cam_proc_tuple = self.single_camera_dict[camera_name]
#         except KeyError:
#             print("ERROR: Camera already not running")
#             return
#         self.close_camera(cam_proc_tuple[0], cam_proc_tuple[1])

#     def close_camera(self, camera, camera_process):
#         print("INFO: Closing \"{}\" . . .".format(camera.camera_name))
#         try:
#             self.rover_close_camera(camera)
#         except Exception as e:
#             print(e)
#             print(
#                 "HINT: Rover camera control node did not respond. Is the rover connected?")
#         self.base_close_camera(camera, camera_process)

#     def rover_close_camera(self, camera):
#         print("INFO: Signal rover to close \"{}\" . . .".format(camera.camera_name))
        
#         # Create a request object
#         req = CameraControl.Request()
#         req.camera = camera
#         req.kill = True
#         req.screenshot = False
#         req.cleanup = False
#         req.calibrate = False

#         # Call the service asynchronously
#         try:
#             future = self.camera_control.call_async(req)
#             future.add_done_callback(self.camera_control_response_callback)  # Add a callback to handle the response
#             self.get_logger().info("Service call made, waiting for response...")
#         except Exception as e:
#             self.get_logger().error(f"Service call failed: {e}")

#     def camera_control_response_callback(self, future):
#         try:
#             response = future.result()  # Get the result of the service call
#             if response:
#                 self.get_logger().info(f"Camera control response: {response}")
#             else:
#                 self.get_logger().error("Camera control service did not respond properly.")
#         except Exception as e:
#             self.get_logger().error(f"Error in service response: {e}")


#     def cleanup(self):
#         print("Closing all camera processes . . .")
#         self.close_all_cameras()

#     def handle_camera_cleanup(self, req):
#         if req.cleanup:
#             self.cleanup()
#         return False, None

#     def handler_stop_signals(self, signum, frame):
#         print("Closing all camera processes . . .")
#         self.close_all_cameras()



# def main(args=None):#TODO compare and fix
#     rclpy.init(args=args)

#     camemra_handler = CameraHandler()

#     rclpy.spin(camemra_handler)

#     camemra_handler.destroy_node()
#     rclpy.shutdown()



# if __name__ == '__main__':
#     main()


import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rover_msgs.msg import DeviceList, Camera
from rover_msgs.srv import CameraControl
from subprocess import Popen, PIPE
from .html_templates import *
from .dev_name_map import BASE_DEV_NAME_MAP, ROVER_DEV_NAME_MAP
import threading
import os
import signal
import glob
import time
from queue import Queue

DEV_UPDATE_PERIOD = 2  # seconds
NUM_OF_CHANNEL = 5

class CameraHandler(Node):

    def __init__(self):
        Node.__init__(self, 'camera_handler')

        # Subscribe to the base home gui
        self.single_camera_subscriber = self.create_subscription(String, '/launch_single_camera', self.launch_single_camera, 1)
        self.close_single_camera_subscriber = self.create_subscription(String, '/close_single_camera', self.close_single_camera, 1)

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

        # Camera control service
        self.camera_control = self.create_client(CameraControl, 'camera_control')
        self.create_service(CameraControl, 'camera_cleanup', self.handle_camera_cleanup)

        signal.signal(signal.SIGINT, self.handler_stop_signals)
        signal.signal(signal.SIGTERM, self.handler_stop_signals)

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
        cam_scripts_path = os.path.expanduser('~') + '/scripts/camera/'
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

    def launch_single_camera(self, msg):
        channel = self.get_available_channel()
        self.get_logger().info(f"Got to launch_single_camera on channel {channel}")
        if channel is None:
            print("ERROR: No available channels")
            return

        camera_name = msg.data
        if self.is_camera_busy(camera_name):
            print("ERROR: Camera \"{}\" is busy".format(camera_name))
            return

        single_camera = Camera()
        single_camera.client_address = "10.0.0.4"
        single_camera.camera_name = camera_name
        single_camera.channel = channel

        self.single_cameras.append(single_camera)

        single_camera_process = None

        print("Launching camera into launch_camera")
        # Launching camera on a separate thread
        # process_queue = Queue()
        
        script = self.launch_camera_script.format(single_camera.channel, self.brightness, self.contrast)

        # Now, pass the script to the thread
        single_camera_process=self.launch_camera(single_camera, single_camera_process, script)

        # Wait for the thread to return the process
        # single_camera_process = process_queue.get()  # This blocks until the process is returned
        self.single_camera_dict[camera_name] = (single_camera, single_camera_process)
        self.get_logger().info(f"INFO: single camera process, which is used for closing the camera, is {single_camera_process}")
        
        if single_camera_process:
            self.single_camera_dict[camera_name] = (single_camera, single_camera_process)

        
    def launch_camera(self, camera, process, script):
        self.get_logger().info("INFO: Launching \"{}\" . . .".format(camera))
        try:
            self.rover_launch_camera(camera)
        except Exception as e:
            self.get_logger().info("HINT: Rover camera control node did not respond. Is the rover connected?")
            return process

        # Now use threading for launching camera in background
        process = self.base_launch_camera(camera, process, script)

        # Put the process in the queue so the main thread can access it
        # process_queue.put(process)
        return process
    
    def rover_launch_camera(self, camera):
        print(camera)
        # Create a request object
        req = CameraControl.Request()
        print("Created camera control object")
    
        # Call the service asynchronously
        req.camera = camera
        print("Set camera attribute")
        req.site_name = "site-1"
        print("Set sitename attribute")
        req.kill = False
        print("Set kill attribute")
        req.screenshot = False
        print("Set screenshot attribute")
        req.cleanup = False
        print("Set cleanup attribute")
        req.calibrate = False            
        print("finish camera control object")
        print("Set calibrate attribute")

        try:
            future = self.camera_control.call_async(req)
            future.add_done_callback(self.camera_control_response_callback)  # Add a callback to handle the response
            self.get_logger().info("Service call made, waiting for response...")

        except Exception as e:
            print("Error calling the service")
            self.get_logger().error(f"Service call failed: {e}")

    def base_launch_camera(self, camera, process, script):
        script = self.launch_camera_script.format(camera.channel, self.brightness, self.contrast)

        if not process or process.poll():
            self.get_logger().info(f"Trying to launch script: {script}")

            # Launch the camera process in a separate thread
            # threading.Thread(target=self.start_camera_process, args=(script, camera)).start()
            process = self.start_camera_process(script, camera)

        self.channel_dict[camera.channel] = True
        self.busy_camera_list.append(camera.camera_name)
        return process

    def start_camera_process(self, script, camera):
        try:
            self.get_logger().info(f"Attempting to start camera process with command: {script}")

            # Start the process using Popen
            process = Popen(script, shell=True, stdout=PIPE, stderr=PIPE, preexec_fn=os.setsid)

            # Start reading the output in another thread
            threading.Thread(target=self.read_output, args=(process, camera)).start()

            # Wait for the process to finish
            process.wait()
            return process
        except Exception as e:
            self.get_logger().error(f"Failed to start camera process: {e}")
            return None

    def read_output(self, process, camera):
        stdout, stderr = process.communicate()
        if stdout:
            self.get_logger().info(f"Camera Process Output for {camera.camera_name}: {stdout.decode()}")
        if stderr:
            self.get_logger().error(f"Camera Process Error for {camera.camera_name}: {stderr.decode()}")

    def base_close_camera(self, camera, camera_process):
        try:
            self.busy_camera_list.remove(camera.camera_name)
        except ValueError:
            self.get_logger().error("Camera not found in busy list")
            pass

        try:
            self.channel_dict[camera.channel] = False
        except:
            self.get_logger().error("Camera not found in channel dict")
            pass

        if camera_process:
            try:
                self.get_logger().info("INFO: Attempting to close camera process at base...")
                os.killpg(os.getpgid(camera_process.pid), signal.SIGTERM)
                self.get_logger().info(f"INFO: Base process for \"{camera.camera_name}\" closed!")
            except Exception as e:
                self.get_logger().error(f"Failed to kill process: {e}")
        else:
            self.get_logger().error("ERROR: Base camera process already closed")

    def close_all_cameras(self):
        print("INFO: Closing all cameras . . .")
        self.close_single_cameras()

    def close_single_camera(self, msg):
        self.get_logger().info("Got to close_single_camera")
        camera_name = msg.data
        try:
            cam_proc_tuple = self.single_camera_dict[camera_name]
        except KeyError:
            print("ERROR: Camera already not running")
            return
        self.close_camera(cam_proc_tuple[0], cam_proc_tuple[1])

    def close_camera(self, camera, camera_process):
        print(f"INFO: Closing \"{camera.camera_name}\" . . .")
        try:
            self.rover_close_camera(camera)
        except Exception as e:
            print(e)
            print("HINT: Rover camera control node did not respond. Is the rover connected?")
        self.base_close_camera(camera, camera_process)

    def rover_close_camera(self, camera):
        print("INFO: Signal rover to close \"{}\" . . .".format(camera.camera_name))

        # Create a request object
        req = CameraControl.Request()
        req.camera = camera
        req.kill = True
        req.screenshot = False
        req.cleanup = False
        req.calibrate = False

        try:
            future = self.camera_control.call_async(req)
            future.add_done_callback(self.camera_control_response_callback)
            self.get_logger().info("Service call made, waiting for response...")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def camera_control_response_callback(self, future):
        try:
            response = future.result()
            if response:
                self.get_logger().info(f"Camera control response: {response}")
            else:
                self.get_logger().error("Camera control service did not respond properly.")
        except Exception as e:
            self.get_logger().error(f"Error in service response: {e}")

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
    camera_handler = CameraHandler()
    rclpy.spin(camera_handler)
    camera_handler.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
