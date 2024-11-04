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

class FiducialsNode : public rclcpp::Node {
public:
    FiducialsNode()
        : Node("fiducials_node") {
        // Initialize subscribers, publishers, and other components
        image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "image_raw", 10, std::bind(&FiducialsNode::imageCallback, this, std::placeholders::_1));

        // Initialize other components as needed
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv::Mat image;
        try {
            // Convert the ROS image message to an OpenCV image
            image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Process the image to detect fiducials
        std::vector<int> marker_ids;
        std::vector<std::vector<cv::Point2f>> marker_corners;
        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(
            cv::aruco::DICT_6X6_250);
        cv::aruco::detectMarkers(image, dictionary, marker_corners, marker_ids);

        if (!marker_ids.empty()) {
            cv::aruco::drawDetectedMarkers(image, marker_corners, marker_ids);
            // Further processing, pose estimation, etc.
        }

        // Optionally publish processed image or other data
    }

    // Member variables
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    // Add publishers, services, etc., as needed
};

// Utility functions

// Calculate Euclidean distance between two points
static double calculateDistance(const cv::Point2f& p1, const cv::Point2f& p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return std::sqrt(dx * dx + dy * dy);
}

// Compute area of a fiducial using Heron's formula
static double calculateFiducialArea(const std::vector<cv::Point2f>& points) {
    const cv::Point2f& p0 = points.at(0);
    const cv::Point2f& p1 = points.at(1);
    const cv::Point2f& p2 = points.at(2);
    const cv::Point2f& p3 = points.at(3);

    double a1 = calculateDistance(p0, p1);
    double b1 = calculateDistance(p0, p3);
    double c1 = calculateDistance(p1, p3);

    double a2 = calculateDistance(p1, p2);
    double b2 = calculateDistance(p2, p3);
    double c2 = c1;

    double s1 = (a1 + b1 + c1) / 2.0;
    double s2 = (a2 + b2 + c2) / 2.0;

    double area1 = std::sqrt(s1 * (s1 - a1) * (s1 - b1) * (s1 - c1));
    double area2 = std::sqrt(s2 * (s2 - a2) * (s2 - b2) * (s2 - c2));

    return area1 + area2;
}

// Estimate reprojection error
static double getReprojectionError(
    const std::vector<cv::Point3f>& object_points,
    const std::vector<cv::Point2f>& image_points,
    const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs,
    const cv::Vec3d& rvec,
    const cv::Vec3d& tvec) {

    std::vector<cv::Point2f> projected_points;

    cv::projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs, projected_points);

    // Calculate RMS image error
    double total_error = 0.0;
    for (size_t i = 0; i < object_points.size(); ++i) {
        double error = calculateDistance(image_points[i], projected_points[i]);
        total_error += error * error;
    }
    return std::sqrt(total_error / static_cast<double>(object_points.size()));
}

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
