#include <cassert>
#include <chrono> // In ROS2, it’s common to use C++'s <chrono> library

#include "FiducialsNode.h"

#include <rclcpp/rclcpp.hpp>

// TF2 and transformations
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// ROS 2 messages
#include <visualization_msgs/msg/marker.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <rover_msgs/msg/fiducial_data.hpp>

// OpenCV
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>

#include <list>
#include <string>
#include <sstream>
#include <vector>

// Main function
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // Create an instance of FiducialsNode
    auto node = std::make_shared<FiducialsNode>();

    RCLCPP_INFO(node->get_logger(), "Starting ArUco detection node");

    // Keep the node running
    rclcpp::spin(node);

    // Shutdown ROS and clean up resources
    rclcpp::shutdown();
    return 0;
}
