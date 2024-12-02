import os
import rclpy
from GPSTools import *
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import NavSatFix, Temperature
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PointStamped
from visualization_msgs.msg import Marker
from std_srvs.srv import Empty
from rover_msgs.srv import WaypointSend, DistHeadingConv, LLToNE, DebugMessages
from rover_msgs.msg import WaypointNav, NavStatus, RoverStateSingleton, PathPlanningWaypoints
from PyQt5.QtWidgets import QWidget, QVBoxLayout
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

class CoordinateInput(Node):
    """
    ROS 2 Node for managing GPS waypoints, visualization, and autonomous navigation
    using GUI interactions.
    """

    def __init__(self):
        super().__init__('coordinate_input')

        # QoS Profile for reliable communication
        qos_profile = QoSProfile(depth=10)

        # Initialize variables and file paths
        self.file_to_write = os.path.join(
            os.getcwd(), 'waypoints_to_be_sent_gps.txt'
        )
        self.delimiter = "   "
        self.checkpoint = None
        self.current_coordinate = None
        self.goal_point = None
        self.planned_path = []
        self.num_waypoints = 0

        # Initialize Publishers
        self.pub_mark_wp = self.create_publisher(Marker, '/wp_marker', qos_profile)
        self.pub_path_planner = self.create_publisher(PathPlanningWaypoints, '/path_planning/way_points', qos_profile)

        # Initialize Subscribers
        self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, qos_profile)
        self.create_subscription(NavSatFix, '/ins/lla', self.lla_callback, qos_profile)
        self.create_subscription(WaypointNav, '/waypoint_nav', self.waypoint_nav_callback, qos_profile)
        self.create_subscription(Twist, '/auto_vel', self.auto_vel_callback, qos_profile)
        self.create_subscription(NavStatus, '/nav_status', self.nav_status_callback, qos_profile)
        self.create_subscription(PointStamped, '/mapviz/clicked_point', self.click_point_callback, qos_profile)
        self.create_subscription(FiducialData, '/fiducial_data', self.fiducial_callback, qos_profile)
        self.create_subscription(Temperature, '/zed/temperature/imu', self.zed_temperature_callback, qos_profile)
        self.create_subscription(Path, '/path_planning/smoothed_path', self.planned_path_callback, qos_profile)

        # Initialize Service Clients
        self.srv_send_wps = self.create_client(WaypointSend, 'SendWaypoints')
        self.srv_conv_dh = self.create_client(DistHeadingConv, 'ConvDistHeading')
        self.srv_ll_to_ne = self.create_client(LLToNE, 'LLToNE')
        self.srv_reset_auto_wp = self.create_client(Empty, 'ResetAutoWP')

        # Timer for periodic debug updates
        self.debug_timer = self.create_timer(0.5, self.debug_messages_callback)

        # Set up GUI and plotting
        self.setup_ui()

        self.get_logger().info('CoordinateInput Node Initialized')

    def setup_ui(self):
        """
        Initializes the GUI layout for waypoint visualization using Matplotlib.
        """
        # Create the Matplotlib figure and canvas
        self.figure = Figure(tight_layout=True)
        self.waypoint_plotter = FigureCanvas(self.figure)
        nav_toolbar = NavigationToolbar(self.waypoint_plotter, None)
        layout = QVBoxLayout()
        layout.addWidget(nav_toolbar)
        layout.addWidget(self.waypoint_plotter)

    def debug_messages_callback(self):
        """
        Periodically logs debug messages or performs a debug operation.
        """
        if self.srv_send_wps.wait_for_service(timeout_sec=1.0):
            try:
                response = self.srv_send_wps.call(WaypointSend.Request())
                self.get_logger().info('Debug Message Callback Triggered')
            except Exception as e:
                self.get_logger().error(f'Debug service call failed: {e}')

    def odom_callback(self, msg):
        """
        Callback to process odometry data.
        """
        self.get_logger().info(f'Received odometry: {msg}')

    def lla_callback(self, msg):
        """
        Callback to process GPS data (latitude and longitude).
        """
        self.current_coordinate = (msg.latitude, msg.longitude)

    def waypoint_nav_callback(self, msg):
        """
        Callback to process waypoint navigation data.
        """
        self.num_waypoints = len(msg.waypoints_lat)
        self.get_logger().info(f'Number of waypoints: {self.num_waypoints}')

    def auto_vel_callback(self, msg):
        """
        Callback to process velocity data.
        """
        self.get_logger().info(f'Linear velocity: {msg.linear.x}, Angular velocity: {msg.angular.z}')

    def nav_status_callback(self, msg):
        """
        Callback to process navigation status updates.
        """
        self.get_logger().info(f'Navigation status: {msg.state}')

    def click_point_callback(self, msg):
        """
        Callback to handle a point clicked on the map (e.g., for adding a waypoint).
        """
        lat = msg.point.y
        lon = msg.point.x
        self.write_coordinates(lat, lon)
        self.refresh_waypoints()
        self.get_logger().info(f'Clicked point added as waypoint: Latitude={lat}, Longitude={lon}')

    def fiducial_callback(self, msg):
        """
        Callback to process fiducial marker data.
        """
        self.get_logger().info(f'Fiducial data received: Distance={msg.dist_to_fiducial}, Angle Offset={msg.angle_offset}')

    def zed_temperature_callback(self, msg):
        """
        Callback to process temperature data from the ZED camera.
        Converts Celsius to Fahrenheit and logs the information.
        """
        temp_f = (msg.temperature * 9.0 / 5.0) + 32
        self.get_logger().info(f'ZED Temperature: {temp_f:.2f} F')

    def planned_path_callback(self, msg):
        """
        Callback to process the planned path from the path planner.
        Updates the internal planned path list and removes duplicate points.
        """
        # Extract path points
        self.planned_path = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]

        # Remove duplicate points
        self.planned_path = [p for i, p in enumerate(self.planned_path) if i == 0 or p != self.planned_path[i - 1]]

        # Log the number of points received
        self.get_logger().info(f'Planned path updated with {len(self.planned_path)} points.')

    def write_coordinates(self, lat, lon):
        """
        Writes a new coordinate (latitude, longitude) to the waypoint file.
        """
        try:
            with open(self.file_to_write, 'a') as f:
                f.write(f'{lat:.6f}{self.delimiter}{lon:.6f}\\n')
            self.get_logger().info(f'Waypoint written: Latitude={lat}, Longitude={lon}')
        except Exception as e:
            self.get_logger().error(f'Failed to write waypoint: {e}')

    def refresh_waypoints(self):
        """
        Reads waypoints from the file and updates the GUI or internal state.
        """
        try:
            with open(self.file_to_write, 'r') as f:
                waypoints = f.read()
                self.get_logger().info(f'Refreshed waypoints from file:\n{waypoints}')
        except FileNotFoundError:
            self.get_logger().warn(f'Waypoint file not found. Creating a new file: {self.file_to_write}')
            open(self.file_to_write, 'w').close()

    def clear_waypoints_on_startup(self):
        """
        Clears all waypoints on startup to avoid phantom coordinates from previous runs.
        """
        open(self.file_to_write, 'w').close()
        self.get_logger().info('Cleared all waypoints on startup.')

    def update_plot(self):
        """
        This Function plots the GPS waypoints for the goal, start, search pattern
        and rover's current position.
        """
        self.figure.clear()
        ax = self.figure.add_subplot(111)

        # Plot rover's current position
        if self.current_coordinate:
            ax.plot(self.current_coordinate[1], self.current_coordinate[0], color='r', marker='^', label='Current Position')

        # Plot checkpoints and waypoints
        if self.checkpoint:
            ax.plot(self.checkpoint[1], self.checkpoint[0], color='b', marker='o', label='Checkpoint')
        if self.goal_point:
            ax.plot(self.goal_point[1], self.goal_point[0], color='g', marker='o', label='Goal Point')

        # Plot planned path
        if self.planned_path:
            path_lon, path_lat = zip(*self.planned_path)
            ax.plot(path_lon, path_lat, 'k--', label='Planned Path')

        # Configure plot
        ax.set_xlabel('Longitude')
        ax.set_ylabel('Latitude')
        ax.set_title('GPS Waypoints')
        ax.grid(True)
        ax.legend()
        ax.autoscale()
        ax.axis('equal')

        # Redraw the plot
        self.waypoint_plotter.draw()

    def on_send_click(self):
        """
        Sends the waypoints to the waypoint manager via a ROS 2 service.
        """
        if not self.planned_path:
            self.get_logger().warn('No waypoints to send; waypoint queue is empty.')
            return

        try:
            # Extract latitude and longitude from waypoints
            wp_lat, wp_lon = zip(*self.planned_path)
            request = WaypointSend.Request()
            request.wp_lat = list(wp_lat)
            request.wp_lon = list(wp_lon)

            # Send request to the service
            if self.srv_send_wps.wait_for_service(timeout_sec=2.0):
                response = self.srv_send_wps.call(request)
                self.get_logger().info(f'Waypoints sent successfully: {response}')
            else:
                self.get_logger().error('WaypointSend service is not available.')
        except Exception as e:
            self.get_logger().error(f'Failed to send waypoints: {e}')

    def on_send_planned_path_click(self):
        """
        Sends the planned path to the waypoint manager via a ROS 2 service.
        """
        if not self.planned_path:
            self.get_logger().warn('No planned path to send; the planned path is empty.')
            return

        try:
            wp_lat, wp_lon = zip(*self.planned_path)
            request = WaypointSend.Request()
            request.wp_lat = list(wp_lat)
            request.wp_lon = list(wp_lon)

            # Call the service
            if self.srv_send_wps.wait_for_service(timeout_sec=2.0):
                response = self.srv_send_wps.call(request)
                self.get_logger().info(f'Planned path sent successfully: {response}')
            else:
                self.get_logger().error('WaypointSend service is not available.')
        except Exception as e:
            self.get_logger().error(f'Failed to send planned path: {e}')

    def on_auto_click(self):
        """
        Toggles the autonomous mode by calling a ROS 2 service.
        """
        try:
            if self.srv_reset_auto_wp.wait_for_service(timeout_sec=2.0):
                self.srv_reset_auto_wp.call(Empty.Request())
                self.get_logger().info('Autonomous mode toggled successfully.')
            else:
                self.get_logger().error('ResetAutoWP service is not available.')
        except Exception as e:
            self.get_logger().error(f'Failed to toggle autonomous mode: {e}')

    def on_reset_click(self):
        """
        Resets the waypoint queue and clears all waypoints.
        """
        try:
            if self.srv_reset_auto_wp.wait_for_service(timeout_sec=2.0):
                self.srv_reset_auto_wp.call(Empty.Request())
                self.planned_path = []
                self.clear_waypoints_on_startup()
                self.get_logger().info('Waypoint queue reset and cleared successfully.')
            else:
                self.get_logger().error('ResetAutoWP service is not available.')
        except Exception as e:
            self.get_logger().error(f'Failed to reset waypoints: {e}')

    def on_coordinate_enter(self, lat: float, lon: float):
        """
        Handles the addition of a new coordinate from user input.
        """
        if not (-90 <= lat <= 90 and -180 <= lon <= 180):
            self.get_logger().warn('Invalid GPS coordinates. Latitude must be between -90 and 90, and longitude between -180 and 180.')
            return

        self.write_coordinates(lat, lon)
        self.refresh_waypoints()
        self.get_logger().info(f'Added new waypoint: Latitude={lat}, Longitude={lon}')

    def convert_minsec_to_float(self, degrees: float, minutes: float = 0, seconds: float = 0) -> float:
        """
        Converts degrees, minutes, and seconds to a decimal degree format.
        """
        return degrees + (minutes / 60) + (seconds / 3600)

    def on_clear_waypoints(self):
        """
        Clears all waypoints from the waypoint file.
        """
        try:
            open(self.file_to_write, 'w').close()
            self.planned_path = []
            self.get_logger().info('All waypoints cleared.')
        except Exception as e:
            self.get_logger().error(f'Failed to clear waypoints: {e}')

    def on_clear_planned_path(self):
        """
        Clears the planned path.
        """
        self.planned_path = []
        self.get_logger().info('Planned path cleared.')

    def on_store_checkpoint(self):
        """
        Stores the rover's current location as a checkpoint.
        """
        if not self.current_coordinate:
            self.get_logger().warn('Cannot store checkpoint; current GPS location is unavailable.')
            return

        self.checkpoint = self.current_coordinate
        self.get_logger().info(f'Checkpoint stored: Latitude={self.checkpoint[0]}, Longitude={self.checkpoint[1]}')

    def on_heading_conversion(self, distance: float, heading: float):
        """
        Converts a distance and heading into a GPS coordinate.
        """
        if not self.srv_conv_dh.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('DistHeadingConv service is not available.')
            return

        try:
            request = DistHeadingConv.Request()
            request.distance = distance
            request.heading = heading

            response = self.srv_conv_dh.call(request)
            self.get_logger().info(f'Converted heading and distance to coordinates: Latitude={response.lat}, Longitude={response.lon}')
        except Exception as e:
            self.get_logger().error(f'Failed to convert heading and distance: {e}')
    
    def on_refresh_click(self):
        """
        Refreshes waypoints from the waypoint file and updates the internal state.
        """
        self.refresh_waypoints()
        self.get_logger().info('Waypoints refreshed from the file.')

    def on_delete_last_waypoint(self):
        """
        Deletes the last waypoint from the waypoint file.
        """
        try:
            # Read all waypoints from the file
            with open(self.file_to_write, 'r') as infile:
                waypoints = infile.readlines()

            if not waypoints:
                self.get_logger().warn('No waypoints to delete.')
                return

            # Write back all but the last waypoint
            with open(self.file_to_write, 'w') as outfile:
                outfile.writelines(waypoints[:-1])

            self.refresh_waypoints()
            self.get_logger().info('Last waypoint deleted.')
        except Exception as e:
            self.get_logger().error(f'Failed to delete last waypoint: {e}')

    def clear_planned_path_callback(self, _):
        """
        Callback to reset the clear planned path check after a delay.
        """
        self.clear_planned_path_check = False
        self.get_logger().info('Clear planned path check reset.')

    def update_base_sat_num(self, msg):
        """
        Updates the number of base satellites from GPS data.
        """
        self.get_logger().info(f'Number of Base Satellites: {msg.numSV}')

    def update_rover_sat_num(self, msg):
        """
        Updates the number of rover satellites and GPS accuracy from GPS data.
        """
        self.get_logger().info(f'Number of Rover Satellites: {msg.numSV}, GPS Accuracy: {msg.hAcc}')
        if msg.hAcc > 1:
            self.get_logger().warn('GPS Accuracy is poor (greater than 1).')
        elif msg.hAcc > 0.05:
            self.get_logger().info('GPS Accuracy is moderate (between 0.05 and 1).')
        else:
            self.get_logger().info('GPS Accuracy is good (less than 0.05).')

    def odom_slot(self, msg):
        """
        Processes odometry data to calculate linear and angular velocity.
        """
        linear = msg.twist.twist.linear
        angular = msg.twist.twist.angular
        lin_velocity = (linear.x**2 + linear.y**2 + linear.z**2)**0.5
        ang_velocity = (angular.x**2 + angular.y**2 + angular.z**2)**0.5

        lin_velocity_str = "0.0" if lin_velocity < 0.03 else f"{lin_velocity:.1f}"
        ang_velocity_str = "0.0" if ang_velocity < 0.03 else f"{ang_velocity:.1f}"

        self.get_logger().info(f'Real Linear Velocity: {lin_velocity_str}, Angular Velocity: {ang_velocity_str}')

    def lla_slot(self, msg):
        """
        Updates the current GPS coordinates from LLA data.
        """
        self.rover_lat = msg.latitude
        self.rover_lon = msg.longitude
        self.get_logger().info(f'Updated Rover GPS: Latitude={self.rover_lat}, Longitude={self.rover_lon}')

    def waypoint_nav_slot(self, msg):
        """
        Updates the internal state with new waypoints and navigation details.
        """
        if msg.waypoints_lat:
            self.goal_point = (msg.waypoints_lat[0], msg.waypoints_lon[0])

        self.num_waypoints = len(msg.waypoints_lat)
        if self.num_waypoints > 1:
            self.search_pattern = np.array([msg.waypoints_lat, msg.waypoints_lon]).T

        dist = msg.distance
        dist_north = msg.distance_north
        dist_east = msg.distance_east
        heading_offset = msg.heading_offset[1]

        self.get_logger().info(f'Distance to Goal: {dist:.1f}, N: {dist_north:.1f}, E: {dist_east:.1f}, Heading Offset: {heading_offset:.1f}')

    def auto_vel_slot(self, msg):
        """
        Updates internal state with the autonomous velocity data.
        """
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        lin_str = f"{linear_velocity:.1f}" if linear_velocity != 0 else "0.0"
        ang_str = f"{angular_velocity:.1f}" if angular_velocity != 0 else "0.0"

        self.get_logger().info(f'Autonomous Velocity: Linear={lin_str}, Angular={ang_str}')

    def planned_path_slot(self, msg):
        """
        Updates the planned path based on the received message.
        """
        # Extract the planned path points
        self.planned_path = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]

        # Remove duplicates in the path
        self.planned_path = [p for i, p in enumerate(self.planned_path) if i == 0 or p != self.planned_path[i - 1]]

        # Log the planned path update
        self.get_logger().info(f'Planned path updated with {len(self.planned_path)} points.')

    def on_clear_waypoints_timer(self, _):
        """
        Resets the clear waypoint check after a timer event.
        """
        self.clear_check = False
        self.get_logger().info('Clear waypoints check reset.')

    def clear_waypoints_on_startup(self):
        """
        Clears all waypoints during startup to ensure no residual data is present.
        """
        try:
            open(self.file_to_write, 'w').close()
            self.get_logger().info('Waypoints cleared on startup.')
        except Exception as e:
            self.get_logger().error(f'Failed to clear waypoints on startup: {e}')

    def init_marker(self):
        """
        Initializes the marker message used for waypoint visualization.
        """
        self.marker_msg = Marker()
        self.marker_msg.header.frame_id = '/origin'
        self.marker_msg.ns = 'basic_shapes'
        self.marker_msg.id = 0
        self.marker_msg.type = Marker.SPHERE
        self.marker_msg.action = Marker.ADD
        self.marker_msg.pose.position.x = 0.0
        self.marker_msg.pose.position.y = 0.0
        self.marker_msg.pose.position.z = 0.0
        self.marker_msg.pose.orientation.x = 0.0
        self.marker_msg.pose.orientation.y = 0.0
        self.marker_msg.pose.orientation.z = 0.0
        self.marker_msg.pose.orientation.w = 1.0
        self.marker_msg.scale.x = 1.0
        self.marker_msg.scale.y = 1.0
        self.marker_msg.scale.z = 1.0
        self.marker_msg.color.r = 1.0
        self.marker_msg.color.g = 0.0
        self.marker_msg.color.b = 0.0
        self.marker_msg.color.a = 1.0
        self.get_logger().info('Marker initialized.')

    def draw_curve(self, p1, p2):
        """
        Draws a curve between two GPS points using hyperbolic cosine interpolation.
        """
        a = (p2[1] - p1[1]) / (np.cosh(p2[0]) - np.cosh(p1[0]))
        b = p1[1] - a * np.cosh(p1[0])
        x = np.linspace(p1[0], p2[0], 100)
        y = a * np.cosh(x) + b
        self.get_logger().info(f'Curve drawn between points {p1} and {p2}.')
        return x, y

    def shutdown_plugin(self):
        """
        Cleans up resources during node shutdown.
        """
        self.get_logger().info('Shutting down plugin.')
        self.destroy_subscribers()
        self.destroy_publishers()
        self.destroy_clients()
        self.destroy_timers()
        rclpy.shutdown()

    def destroy_subscribers(self):
        """
        Unsubscribes all active ROS topics.
        """
        self.get_logger().info('Unsubscribing from all topics.')

    def destroy_publishers(self):
        """
        Cleans up all publishers.
        """
        self.get_logger().info('Destroying publishers.')

    def destroy_clients(self):
        """
        Cleans up all service clients.
        """
        self.get_logger().info('Destroying service clients.')

    def destroy_timers(self):
        """
        Cleans up all active timers.
        """
        self.get_logger().info('Destroying timers.')

    def on_mark_gps(self, latitude: float, longitude: float):
        """
        Adds a GPS marker to the map for the given latitude and longitude.
        """
        try:
            if not (-90 <= latitude <= 90 and -180 <= longitude <= 180):
                self.get_logger().warn('Invalid GPS coordinates for marker.')
                return

            # Update marker position
            self.marker_msg.pose.position.x = longitude
            self.marker_msg.pose.position.y = latitude
            self.marker_msg.action = Marker.ADD

            # Publish the marker
            self.pub_mark_wp.publish(self.marker_msg)
            self.get_logger().info(f'GPS Marker added: Latitude={latitude}, Longitude={longitude}')
        except Exception as e:
            self.get_logger().error(f'Failed to add GPS marker: {e}')

    def on_unmark_gps(self):
        """
        Clears all GPS markers from the map.
        """
        try:
            self.marker_msg.action = Marker.DELETEALL
            self.pub_mark_wp.publish(self.marker_msg)
            self.get_logger().info('All GPS markers cleared.')
        except Exception as e:
            self.get_logger().error(f'Failed to clear GPS markers: {e}')

    def on_store_checkpoint(self):
        """
        Stores the rover's current GPS coordinates as a checkpoint.
        """
        if not self.current_coordinate:
            self.get_logger().warn('Current GPS location is unavailable; cannot store checkpoint.')
            return

        self.checkpoint = self.current_coordinate
        self.get_logger().info(f'Checkpoint stored: Latitude={self.checkpoint[0]}, Longitude={self.checkpoint[1]}')

    def on_return_to_checkpoint(self):
        """
        Sends the rover to the last stored checkpoint.
        """
        if not self.checkpoint:
            self.get_logger().warn('No checkpoint is currently stored.')
            return

        try:
            request = WaypointSend.Request()
            request.wp_lat = [self.checkpoint[0]]
            request.wp_lon = [self.checkpoint[1]]

            if self.srv_send_wps.wait_for_service(timeout_sec=2.0):
                self.srv_send_wps.call(request)
                self.get_logger().info(f'Rover directed to checkpoint: Latitude={self.checkpoint[0]}, Longitude={self.checkpoint[1]}')
            else:
                self.get_logger().error('WaypointSend service is not available.')
        except Exception as e:
            self.get_logger().error(f'Failed to send rover to checkpoint: {e}')

    def get_target_id_from_leg(self, leg: int) -> int:
        """
        Determines the target AR tag ID based on the given leg number.
        """
        if leg <= 3:
            return -1  # Legs 1-3 are not associated with AR tags
        elif 4 <= leg <= 6:
            return leg - 3  # Legs 4-6 use tags 1-3
        elif leg == 7:
            return 4  # Leg 7 uses tag 4
        else:
            self.get_logger().warn(f'Invalid leg number: {leg}')
            return -1

    def on_send_to_path_planner_click(self):
        """
        Sends waypoints to the path planner and waits for the planned path.
        """
        if not self.planned_path:
            self.get_logger().warn('No waypoints to send; waypoint queue is empty.')
            return

        try:
            # Prepare path planning message
            request = PathPlanningWaypoints()
            latitudes, longitudes = zip(*self.planned_path)
            request.latitude_coords = list(latitudes)
            request.longitude_coords = list(longitudes)
            request.lock_order = True

            # Publish the message
            self.pub_path_planner.publish(request)
            self.get_logger().info('Waypoints sent to the path planner; awaiting response...')
        except Exception as e:
            self.get_logger().error(f'Failed to send waypoints to the path planner: {e}')

    def refresh_waypoints(self):
        """
        Refreshes waypoints from the file and updates the internal state.
        """
        try:
            with open(self.file_to_write, 'r') as file:
                waypoints = file.readlines()
                self.get_logger().info(f'Refreshed waypoints from file:\n{waypoints}')
        except FileNotFoundError:
            self.get_logger().warn('Waypoint file not found; creating a new one.')
            open(self.file_to_write, 'w').close()

    def write_coordinates(self, latitude: float, longitude: float):
        """
        Writes a GPS coordinate to the waypoint file.
        """
        try:
            with open(self.file_to_write, 'a') as file:
                file.write(f'{latitude:.6f}{self.delimiter}{longitude:.6f}\\n')
            self.get_logger().info(f'Coordinate written: Latitude={latitude}, Longitude={longitude}')
        except Exception as e:
            self.get_logger().error(f'Failed to write coordinates to file: {e}')

    def planned_path_slot(self, msg):
        """
        Updates the planned path based on a message from the path planner.
        """
        try:
            self.planned_path = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]

            # Remove duplicate points
            self.planned_path = [point for i, point in enumerate(self.planned_path) if i == 0 or point != self.planned_path[i - 1]]

            self.get_logger().info(f'Updated planned path with {len(self.planned_path)} points.')
        except Exception as e:
            self.get_logger().error(f'Failed to update planned path: {e}')

    def clear_planned_path_callback(self, _):
        """
        Resets the `clear_planned_path_check` flag after a timer event.
        """
        self.clear_planned_path_check = False
        self.get_logger().info('Clear planned path flag reset.')

    def clear_wp_callback(self, _):
        """
        Resets the `clear_check` flag after a timer event.
        """
        self.clear_check = False
        self.get_logger().info('Clear waypoint flag reset.')

    def on_clear_planned_path(self):
        """
        Clears the planned path with a confirmation mechanism.
        """
        if not self.clear_planned_path_check:
            self.clear_planned_path_check = True
            self.create_timer(5.0, self.clear_planned_path_callback)
            self.get_logger().warn('Clear planned path? Click again within 5 seconds to confirm.')
        else:
            self.planned_path = []
            self.get_logger().info('Planned path cleared.')

    def on_clear_waypoints(self):
        """
        Clears all waypoints with a confirmation mechanism.
        """
        if not self.clear_check:
            self.clear_check = True
            self.create_timer(5.0, self.clear_wp_callback)
            self.get_logger().warn('Clear all waypoints? Click again within 5 seconds to confirm.')
        else:
            try:
                open(self.file_to_write, 'w').close()
                self.get_logger().info('All waypoints cleared.')
            except Exception as e:
                self.get_logger().error(f'Failed to clear waypoints: {e}')

    def on_delete_last_waypoint(self):
        """
        Deletes the last waypoint from the waypoint file.
        """
        try:
            # Read waypoints from the file
            with open(self.file_to_write, 'r') as file:
                waypoints = file.readlines()

            if not waypoints:
                self.get_logger().warn('No waypoints to delete.')
                return

            # Rewrite all waypoints except the last one
            with open(self.file_to_write, 'w') as file:
                file.writelines(waypoints[:-1])

            self.get_logger().info('Last waypoint deleted.')
        except Exception as e:
            self.get_logger().error(f'Failed to delete last waypoint: {e}')

    def on_refresh_click(self):
        """
        Refreshes the waypoints from the file.
        """
        self.refresh_waypoints()
        self.get_logger().info('Waypoints refreshed.')

    def on_coordinate_enter(self, latitude: float, longitude: float):
        """
        Adds a new coordinate to the waypoint file from user input.
        """
        if not (-90 <= latitude <= 90 and -180 <= longitude <= 180):
            self.get_logger().warn('Invalid GPS coordinates.')
            return

        self.write_coordinates(latitude, longitude)
        self.refresh_waypoints()
        self.get_logger().info(f'New coordinate entered: Latitude={latitude}, Longitude={longitude}')

    def convert_minsec_to_float(self, degrees: float, minutes: float, seconds: float) -> float:
        """
        Converts degrees, minutes, and seconds into decimal degrees.
        """
        decimal_degrees = degrees + (minutes / 60.0) + (seconds / 3600.0)
        self.get_logger().info(f'Converted to decimal degrees: {decimal_degrees}')
        return decimal_degrees

    def update_plot(self):
        """
        Updates the Matplotlib plot with GPS waypoints and rover state.
        """
        try:
            field_of_view = 120
            self.figure.clear()
            ax = self.figure.add_subplot(111)

            # print("starting point: ", self.checkpoint)
            # print("goal point: ", self.goal_point)
            # print("current point: ", self.current_coordinate)
            # print("search pattern: ", self.search_pattern)

            direction_indicating_point1 = GPSTools.heading_distance_to_lat_lon(
                self.current_coordinate, self.current_heading - field_of_view / 2, 10
            )
            direction_indicating_point2 = GPSTools.heading_distance_to_lat_lon(
                self.current_coordinate, self.current_heading + field_of_view / 2, 10
            )

            # plot starting point
            ax.plot(self.checkpoint.lon, self.checkpoint.lat, color="b", marker="o")
            # plot rover's current position and heading/field of view
            ax.plot(
                [self.current_coordinate.lon, direction_indicating_point1.lon],
                [self.current_coordinate.lat, direction_indicating_point1.lat],
                color="r",
            )
            ax.plot(
                [self.current_coordinate.lon, direction_indicating_point2.lon],
                [self.current_coordinate.lat, direction_indicating_point2.lat],
                color="r",
            )
            # ax.plot(self.draw_curve((direction_indicating_point1.lon, direction_indicating_point1.lat), (direction_indicating_point2.lon, direction_indicating_point2.lat)), color='r')
            ax.plot(
                self.current_coordinate.lon,
                self.current_coordinate.lat,
                color="r",
                marker="^",
            )

            # plot goal point
            ax.plot(self.goal_point.lon, self.goal_point.lat, color="g", marker="o")

            # plot search pattern
            ax.plot(self.search_pattern[:, 1], self.search_pattern[:, 0], "k--")

            ax.set_xlabel("longitude")
            ax.set_ylabel("latitude")
            ax.set_title("GPS Waypoints")
            ax.grid(True)
            ax.autoscale()
            ax.axis("equal")

            # Redraw the plot
            self.waypoint_plotter.draw()
            self.get_logger().info('Plot updated.')
        except Exception as e:
            self.get_logger().error(f'Failed to update plot: {e}')
