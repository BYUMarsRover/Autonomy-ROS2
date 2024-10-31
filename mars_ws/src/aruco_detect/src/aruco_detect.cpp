/*
 * Copyright (c) 2017-20, Ubiquity Robotics Inc., Austin Hendrix
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of the FreeBSD Project.
 *
 */

#include <cassert> // Using <cassert> aligns with modern C++ standards, which are emphasized more in ROS2 for compatibility
#include <chrono> // In ROS2, it’s common to use C++'s <chrono> library
#include <unistd.h> // Often remains the same if you need system time functions

#include "FiducialsNode.h"

#include <rclcpp/rclcpp.hpp> // Updated to ROS2

// Packages below are for transformations
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <visualization_msgs/msg/marker.hpp>  // In ROS2, messages (msg), services (srv), and actions (action) are stored in separate folders and .hpp makes it clear that these headers are designed for C++
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
// #include <dynamic_reconfigure/server.hpp>
// TODO: Use ROS2 parameters instead of dynamic reconfigure
// Do we need this? What is its use? This is not available in ROS2 

#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <rover_msgs/msg/fiducial_data.hpp>

// #include "config/DetectorParamsConfig.h" --> should this be DetectorParams.cfg? TODO: JM - I think that they should be .yaml
// we don't have the dynamic reconfigure available & this has to do with that. Need to look into getting that set up 

#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>

#include <list>
#include <string>
#include <sstream>
#include <vector>

using namespace std;
using namespace cv;
using std::vector;
using std::string;

// This set is for the competition tags (two black borders)
static vector<vector<vector<unsigned char>>> COMPETITION_MARKERS_7X7 = {
    {{0, 217, 178, 167}, {207, 128, 0, 0}, {249, 176, 230, 207}, {128, 0, 0, 249},
     {242, 166, 205, 128}, {0, 0, 249, 179}, {134, 207, 128, 0}},
    {{0, 217, 178, 161}, {142, 128, 0, 0}, {233, 160, 230, 78}, {128, 0, 0, 184},
     {194, 166, 205, 128}, {0, 0, 185, 51}, {130, 203, 128, 0}},
    // Additional markers omitted for brevity
};

// This set is for tags with one black border (not competition tags)
static vector<vector<vector<unsigned char>>> COMPETITION_MARKERS_5X5_20_BYTES = {
    {{222, 235, 255, 1}, {254, 207, 191, 1}, {255, 235, 189, 1}, {254, 249, 191, 1}},
    {{222, 234, 110, 1}, {238, 143, 158, 1}, {187, 43, 189, 1}, {188, 248, 187, 1}},
    // Additional markers omitted for brevity
};

/**
  * @brief Return object points for the system centered in a single marker, given the marker length
  */
static void getSingleMarkerObjectPoints(float markerLength, vector<cv::Point3f>& objPoints) {
    CV_Assert(markerLength > 0);

    // Set coordinate system in the middle of the marker, with Z pointing out
    objPoints.clear();
    objPoints.push_back(cv::Point3f(-markerLength / 2.f, markerLength / 2.f, 0));
    objPoints.push_back(cv::Point3f(markerLength / 2.f, markerLength / 2.f, 0));
    objPoints.push_back(cv::Point3f(markerLength / 2.f, -markerLength / 2.f, 0));
    objPoints.push_back(cv::Point3f(-markerLength / 2.f, -markerLength / 2.f, 0));
}

// Euclidean distance between two points
static double dist(const cv::Point2f &p1, const cv::Point2f &p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return std::sqrt(dx * dx + dy * dy);
}

// Compute area in image of a fiducial, using Heron's formula to find the area of two triangles
static double calcFiducialArea(const vector<cv::Point2f> &pts) {
    const cv::Point2f &p0 = pts.at(0);
    const cv::Point2f &p1 = pts.at(1);
    const cv::Point2f &p2 = pts.at(2);
    const cv::Point2f &p3 = pts.at(3);

    double a1 = dist(p0, p1);
    double b1 = dist(p0, p3);
    double c1 = dist(p1, p3);

    double a2 = dist(p1, p2);
    double b2 = dist(p2, p3);
    double c2 = c1;

    double s1 = (a1 + b1 + c1) / 2.0;
    double s2 = (a2 + b2 + c2) / 2.0;

    a1 = std::sqrt(s1 * (s1 - a1) * (s1 - b1) * (s1 - c1));
    a2 = std::sqrt(s2 * (s2 - a2) * (s2 - b2) * (s2 - c2));
    return a1 + a2;
}

// Estimate reprojection error
static double getReprojectionError(const vector<cv::Point3f> &objectPoints,
                                   const vector<cv::Point2f> &imagePoints,
                                   const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs,
                                   const cv::Vec3d &rvec, const cv::Vec3d &tvec) {
    vector<cv::Point2f> projectedPoints;

    cv::projectPoints(objectPoints, rvec, tvec, cameraMatrix, distCoeffs, projectedPoints);

    // Calculate RMS image error
    double totalError = 0.0;
    for (size_t i = 0; i < objectPoints.size(); i++) {
        double error = dist(imagePoints[i], projectedPoints[i]);
        totalError += error * error;
    }
    return std::sqrt(totalError / static_cast<double>(objectPoints.size()));
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<FiducialsNode>();

    RCLCPP_INFO(node->get_logger(), "Starting Aruco detection node");

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
