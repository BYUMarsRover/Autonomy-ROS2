"""
This class provides an interface with the mobility module
"""

from rover_msgs.msg import MobilityGPSWaypoint2Follow, MobilityAutopilotCommand, MobilityVelocityCommands, MobilityArucoAutopilotCommand
from std_srvs.srv import SetBool
import rclpy
from rclpy.node import Node


class DriveControllerAPI(Node):
    
    def __init__(self):
        super().__init__('drive_controller_api')

        self.path_cmds_pub = self.create_publisher(MobilityGPSWaypoint2Follow, '/mobility/waypoint2follow', 10)
        self.autopilot_cmds_pub = self.create_publisher(MobilityAutopilotCommand, "/mobility/autopilot_cmds", 10)
        self.drive_cmds_pub = self.create_publisher(MobilityVelocityCommands, '/mobility/rover_vel_cmds', 10)
        self.aruco_autopilot_cmds_pub = self.create_publisher(MobilityArucoAutopilotCommand, '/mobility/aruco_autopilot_cmds', 10)

        self.path_cmd = MobilityGPSWaypoint2Follow()
        self.autopilot_cmd = MobilityAutopilotCommand()
        self.drive_cmd = MobilityVelocityCommands()
        self.aruco_autopilot_cmd = MobilityArucoAutopilotCommand()

        self.path_manager_enabled = False
        self.autopilot_manager_enabled = False
        self.drive_manager_enabled = False
        self.wheel_manager_enabled = False
        self.aruco_autopilot_manager_enabled = False

    def stop(self):
        self._activate_managers([False]*5)

    def issue_path_cmd(self, lat, lon, yaw=0):
        self._activate_managers([True, True, True, True, False])
        self.path_cmd.latitude = lat
        self.path_cmd.longitude = lon
        self.path_cmds_pub.publish(self.path_cmd)
    
    def issue_autopilot_cmd(self, distance, heading):
        self._activate_managers([False, True, True, True, False])
        self.autopilot_cmd.distance_to_target = distance
        self.autopilot_cmd.course_angle = heading
        self.autopilot_cmds_pub.publish(self.autopilot_cmd)

    def issue_drive_cmd(self, lin_vel, ang_vel):
        self._activate_managers([False, False, True, True, False])
        self.drive_cmd.u_cmd = lin_vel
        self.drive_cmd.omega_cmd = ang_vel
        self.drive_cmds_pub.publish(self.drive_cmd)

    def issue_aruco_autopilot_cmd(self, angle, distance):
        self._activate_managers([False, False, True, True, True])
        self.aruco_autopilot_cmd.distance_to_target = distance
        self.aruco_autopilot_cmd.angle_to_target = angle
        self.aruco_autopilot_cmds_pub.publish(self.aruco_autopilot_cmd)
    
    def _activate_managers(self, enable_list):
        assert len(enable_list) == 5, "enable list is wrong size"
        if enable_list[0] != self.path_manager_enabled:
            self._toggle_enable_path_manager(enable_list[0])

        if enable_list[1] != self.autopilot_manager_enabled:
            self._toggle_enable_autopilot_manager(enable_list[1])

        if enable_list[2] != self.drive_manager_enabled:
            self._toggle_enable_drive_manager(enable_list[2])

        if enable_list[3] != self.wheel_manager_enabled:
            self._toggle_enable_wheel_manager(enable_list[3])

        if enable_list[4] != self.aruco_autopilot_manager_enabled:
            self._toggle_enable_aruco_autopilot_manager(enable_list[4])

    def _toggle_enable_path_manager(self, enable):
        self._toggle_service('/mobility/path_manager/enabled', enable, 'path_manager_enabled')

    def _toggle_enable_autopilot_manager(self, enable):
        self._toggle_service('/mobility/autopilot_manager/enabled', enable, 'autopilot_manager_enabled')

    def _toggle_enable_drive_manager(self, enable):
        self._toggle_service('/mobility/drive_manager/enabled', enable, 'drive_manager_enabled')

    def _toggle_enable_wheel_manager(self, enable):
        self._toggle_service('/mobility/wheel_manager/enabled', enable, 'wheel_manager_enabled')

    def _toggle_enable_aruco_autopilot_manager(self, enable):
        self._toggle_service('/mobility/aruco_autopilot_manager/enabled', enable, 'aruco_autopilot_manager_enabled')

    def _toggle_service(self, service_name, enable, manager_attr):
        client = self.create_client(SetBool, service_name)
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Service {service_name} not available, waiting...')

        request = SetBool.Request()
        request.data = enable

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response.success:
            setattr(self, manager_attr, enable)


def main(args=None):
    rclpy.init(args=args)
    drive_controller_api = DriveControllerAPI()
    rclpy.spin(drive_controller_api)

    drive_controller_api.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()