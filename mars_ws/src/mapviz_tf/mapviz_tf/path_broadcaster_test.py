import rclpy
import threading
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.time import Time
from std_msgs.msg import Header
from nav_msgs.msg import Path as PathMsg
from geometry_msgs.msg import Point, Pose, Quaternion, PoseStamped
from visualization_msgs.msg import Marker #needed for the markers 
from std_msgs.msg import ColorRGBA #needed for markers
from mapviz_tf.lat_lon_meter_convertor import LatLonConvertor

def spin_in_background():
    executor = rclpy.get_global_executor()
    try:
        executor.spin()
    except ExternalShutdownException:
        pass

class PathBroadcasterTest(Node):

    def __init__(self):
        super().__init__('path_test')
        # # Subscriber to "/path_planning/smoothed_path"
        # self.path_planning_sub = self.create_subscription(PathMsg, "/path_planning/smoothed_path", self.path_planning_callback, 10)

        # Publisher to "/mapviz/path"
        self.pub = self.create_publisher(PathMsg, '/mapviz/path', 10)
        self.pointpub = self.create_publisher(Marker, 'visualization_marker', 10)
        self.travel = 0
        self.path_coords = [(-110.793304,38.4065),(-110.791997,38.406606),(-110.790715,38.405936)]

        # Create a timer to periodically send the path (every 1 second)
        self.create_timer(1.0, self.path_send)
        self.create_timer(1.0, self.publish_marker)

        # Pose array to store poses
        self.poses_array = []
        # Latitude and longitude conversion utility
        self.latlonconv = LatLonConvertor()
        self.meters_per_degree_latitude, self.meters_per_degree_longitude = self.latlonconv.get_meters_per_degree_lat_lon()
        # self.get_logger().info(f"Meters per degree (Latitude, Longitude): ({self.meters_per_degree_latitude}, {self.meters_per_degree_longitude})")

    def clean(self):
        """
        This clears the existing path.
        """
        self.poses_array.clear()

    # it would normally be best to have this in a separate file but I'm just testing these, this is not for paths
    # but instead for points. It could probably be used for a path though. In fact, it might be better if it can
    # make a line and not a shape. Code mostly from chatgpt.
    def publish_marker(self): 
        # Create a Marker message
        marker = Marker()
        marker.header.frame_id = "wgs84"  # Replace with your frame of reference - wgs84 allows us to publish in coordinates
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "star"
        marker.id = 0
        marker.type = Marker.LINE_STRIP  # LINE_STRIP connects points in order
        marker.action = Marker.ADD
        marker.scale.x = 0.05  # Line width

        # Set the color (RGBA)
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # Red color, fully opaque

        # Specify the coordinates of the points
        star_coordinates = [
            (0.0, 1.0),    # Point 1
            (0.475, 0.154),# Point 2
            (0.588, -0.809),# Point 3
            (-0.294, -0.404),# Point 4
            (-0.951, 0.309),# Point 5
            (0.0, 2.0)     # Close the loop
        ]

        # path_coords = [(-110.793304,38.4065),(-110.791997,38.406606),(-110.790715,38.405936)]
        self.path_coords = [(coord[0] + 0.000001, coord[1]) for coord in self.path_coords]

        # Add points to the Marker message
        for x, y in self.path_coords:
            point = Point(x=x, y=y)
            marker.points.append(point)

        # Publish the Marker message
        self.pointpub.publish(marker)
        self.get_logger().info("Star marker published!")


    def path_send(self):
        """
        Callback to process and publish the path.
        """
        self.get_logger().info("New path received!")

        # Create a fixed orientation for each pose
        quaternion_msg = Quaternion()
        quaternion_msg.x = 0.0
        quaternion_msg.y = 0.0
        quaternion_msg.z = 0.0
        quaternion_msg.w = 1.0

        # Make a header message
        header_msg = Header()
        # header_msg.seq = msg.header.seq
        header_msg.stamp = self.get_clock().now().to_msg()
        header_msg.frame_id = 'map'

        # Prepare the path message
        path_msg = PathMsg()
        path_msg.header = header_msg

        # Convert lat/lon to x/y for each pose in the path
        # points = [(38.406474,-110.791923),(38.406629,-110.791686),(38.406250,-110.791242),(38.406152,-110.792265)]
        # points = [(5.2,6.8),(52.7,-5.5),(52.7,-30.1),(20.4,-12.8)]
        points = [(0.000,1.000),(0.475,0.154),(0.588,-0.809),(-0.475,-0.154),(-0.588,0.809),(0.000,0.500),(0.294,-0.404),(-0.294,-0.404),(-0.475,0.404),(0.294,0.404)]
        for pose in points:
            # Convert lat/lon to meters
            # point = self.latlonconv.convert_to_meters(pose[1], pose[0])
            point = {
                            "x": pose[0] * 10,
                            "y": pose[1] * 10,#10 is arbitrary because path is smol.
                        }
            
            # Populate Point message with converted coordinates
            point_msg = Point(x=point['x'], y=point['y'], z=1501.0)

            # Populate Pose message
            pose_msg = Pose(position=point_msg, orientation=quaternion_msg)

            # Populate PoseStamped message with header and pose
            pose_stamped_msg = PoseStamped(header=header_msg, pose=pose_msg)

            # Add pose to the array
            self.poses_array.append(pose_stamped_msg)

        # Assign the array of poses to the path message and publish it
        path_msg.poses = self.poses_array
        self.get_logger().info("Publishing path message!")
        self.pub.publish(path_msg)

def main():
    rclpy.init()
    node = PathBroadcasterTest()

    try:
        # Run spin in the main thread
        # node.create_timer(1.0, node.path_send)  # Broadcast every 1 second
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()