#ifndef FIDUCIALS_NODE_H  // Include guard to prevent multiple inclusion
#define FIDUCIALS_NODE_H

#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <rover_msgs/msg/fiducial_data.hpp>
#include <rover_msgs/msg/fiducial_array.hpp>
#include <rover_msgs/msg/fiducial_transform.hpp>
// #include <fiducial_msgs/msg/fiducial_array.hpp>
// #include <fiducial_msgs/msg/fiducial_transform_array.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <boost/algorithm/string.hpp>
#include <map>
#include <vector>

using namespace rclcpp;  // Use the rclcpp namespace directly

class FiducialsNode {
private:
    // Publishers
    Publisher<fiducial_msgs::msg::FiducialArray>::SharedPtr vertices_pub;  
    Publisher<fiducial_msgs::msg::FiducialTransformArray>::SharedPtr pose_pub;

    // Subscribers
    Subscription<std_msgs::msg::String>::SharedPtr caminfo_sub;
    Subscription<std_msgs::msg::String>::SharedPtr ignore_sub;
    image_transport::Subscriber img_sub;

    // Service server
    Service<std_srvs::srv::SetBool>::SharedPtr service_enable_detections;

    // Parameters
    bool publish_images;
    bool enable_detections;
    bool doPoseEstimation;
    bool haveCamInfo;

    int frameNum;
    double fiducial_len;

    cv::Mat cameraMatrix;
    cv::Mat distortionCoeffs;
    
    std::string frameId;
    std::vector<int> ignoreIds;
    std::map<int, double> fiducialLens;

    image_transport::ImageTransport it;

    // OpenCV and ARUCO
    cv::Ptr<cv::aruco::DetectorParameters> detectorParams;
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    
    void handleIgnoreString(const std::string& str);
    void estimatePoseSingleMarkers(const std::vector<int>& ids,
                                    const std::vector<std::vector<cv::Point2f>>& corners,
                                    float markerLength,
                                    const cv::Mat& cameraMatrix,
                                    const cv::Mat& distCoeffs,
                                    std::vector<cv::Vec3d>& rvecs, 
                                    std::vector<cv::Vec3d>& tvecs,
                                    std::vector<double>& reprojectionError);

    void ignoreCallback(const std_msgs::msg::String::SharedPtr msg);
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void camInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    
    bool enableDetectionsCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                                   const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                                   const std::shared_ptr<std_srvs::srv::SetBool::Response> res);

    rcl_interfaces::msg::SetParametersResult parameterCallback(const std::vector<Parameter>& params);

    // COMMENTED OUT CODE BELOW AS dynamic_reconfigure is not available for ROS2
    // dynamic_reconfigure::Server<aruco_detect::DetectorParamsConfig> configServer; 
    // dynamic_reconfigure::Server<aruco_detect::DetectorParamsConfig>::CallbackType callbackType; 

    // NOTE: Parameter callback for dynamic changes. This function is called whenever a parameter is updated at runtime. 
    // Inside this callback, you check which parameters were updated and modify the node’s internal state accordingly.
    // rcl_interfaces::msg::SetParametersResult parameterCallback(const std::vector<rclcpp::Parameter> &params) {
    //     for (const auto &param : params) {
    //         if (param.get_name() == "fiducial_len") {
    //             fiducial_len = param.as_double();
    //         } else if (param.get_name() == "publish_images") {
    //             publish_images = param.as_bool();
    //         } else if (param.get_name() == "enable_detections") {
    //             enable_detections = param.as_bool();
    //         }
    //         // Update other parameters as needed
    //     }
    //     return rcl_interfaces::msg::SetParametersResult{true};
    // }

    public:
    FiducialsNode();  // Constructor

    // Callback methods
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void camInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void ignoreCallback(const std_msgs::msg::String::SharedPtr msg);

    // Service callback
    bool enableDetectionsCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                                   const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                                   const std::shared_ptr<std_srvs::srv::SetBool::Response> res);

    // Parameter callback
    rcl_interfaces::msg::SetParametersResult parameterCallback(const std::vector<Parameter>& params);
};
