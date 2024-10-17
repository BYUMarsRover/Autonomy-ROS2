#ifndef FIDUCIALS_NODE_H  // Include guard to prevent multiple inclusion
#define FIDUCIALS_NODE_H

#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <opencv2/opencv.hpp>
#include <map>
#include <vector>
#include <string>
using namespace std;

class FiducialsNode : public rclcpp::Node {
  private:
    // Publishers in ROS2
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vertices_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pose_pub;
    image_transport::Publisher image_pub;

    // Subscribers in ROS2
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr caminfo_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ignore_sub;
    image_transport::Subscriber img_sub;

    // Service server in ROS2
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_enable_detections;

    // Internal variables
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

    cv::Ptr<aruco::DetectorParameters> detectorParams;
    cv::Ptr<aruco::Dictionary> dictionary;

    void handleIgnoreString(const std::string& str);

    void estimatePoseSingleMarkers(const vector<int> &ids,
                                   const vector<vector<Point2f>> &corners,
                                   float markerLength,
                                   const cv::Mat &cameraMatrix,
                                   const cv::Mat &distCoeffs,
                                   vector<Vec3d>& rvecs, vector<Vec3d>& tvecs,
                                   vector<double>& reprojectionError);

    // Updated callback functions to use ROS2 SharedPtr
    void ignoreCallback(const std_msgs::msg::String::SharedPtr msg);
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void camInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

    bool enableDetectionsCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                  std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    // Parameter callback for dynamic changes in ROS2
    rcl_interfaces::msg::SetParametersResult parameterCallback(const std::vector<rclcpp::Parameter> &params) {
        for (const auto &param : params) {
            if (param.get_name() == "fiducial_len") {
                fiducial_len = param.as_double();
            } else if (param.get_name() == "publish_images") {
                publish_images = param.as_bool();
            } else if (param.get_name() == "enable_detections") {
                enable_detections = param.as_bool();
            }
            // Update other parameters as needed
        }
        return rcl_interfaces::msg::SetParametersResult{true};
    }

  public:
    FiducialsNode();

void configCallback(const std::vector<rclcpp::Parameter> &params) {
    for (const auto &param : params) {
        if (param.get_name() == "adaptiveThreshConstant") {
            detectorParams->adaptiveThreshConstant = param.as_double();
        } else if (param.get_name() == "adaptiveThreshWinSizeMin") {
            detectorParams->adaptiveThreshWinSizeMin = param.as_int();
        } else if (param.get_name() == "adaptiveThreshWinSizeMax") {
            detectorParams->adaptiveThreshWinSizeMax = param.as_int();
        } else if (param.get_name() == "adaptiveThreshWinSizeStep") {
            detectorParams->adaptiveThreshWinSizeStep = param.as_int();
        } else if (param.get_name() == "cornerRefinementMaxIterations") {
            detectorParams->cornerRefinementMaxIterations = param.as_int();
        } else if (param.get_name() == "cornerRefinementMinAccuracy") {
            detectorParams->cornerRefinementMinAccuracy = param.as_double();
        } else if (param.get_name() == "cornerRefinementWinSize") {
            detectorParams->cornerRefinementWinSize = param.as_int();
        } else if (param.get_name() == "errorCorrectionRate") {
            detectorParams->errorCorrectionRate = param.as_double();
        } else if (param.get_name() == "minCornerDistanceRate") {
            detectorParams->minCornerDistanceRate = param.as_double();
        } else if (param.get_name() == "markerBorderBits") {
            detectorParams->markerBorderBits = param.as_int();
        } else if (param.get_name() == "maxErroneousBitsInBorderRate") {
            detectorParams->maxErroneousBitsInBorderRate = param.as_double();
        } else if (param.get_name() == "minDistanceToBorder") {
            detectorParams->minDistanceToBorder = param.as_int();
        } else if (param.get_name() == "minMarkerDistanceRate") {
            detectorParams->minMarkerDistanceRate = param.as_double();
        } else if (param.get_name() == "minMarkerPerimeterRate") {
            detectorParams->minMarkerPerimeterRate = param.as_double();
        } else if (param.get_name() == "maxMarkerPerimeterRate") {
            detectorParams->maxMarkerPerimeterRate = param.as_double();
        } else if (param.get_name() == "minOtsuStdDev") {
            detectorParams->minOtsuStdDev = param.as_double();
        } else if (param.get_name() == "perspectiveRemoveIgnoredMarginPerCell") {
            detectorParams->perspectiveRemoveIgnoredMarginPerCell = param.as_double();
        } else if (param.get_name() == "perspectiveRemovePixelPerCell") {
            detectorParams->perspectiveRemovePixelPerCell = param.as_int();
        } else if (param.get_name() == "polygonalApproxAccuracyRate") {
            detectorParams->polygonalApproxAccuracyRate = param.as_double();
        }
    }
}

void ignoreCallback(const std_msgs::msg::String::SharedPtr msg)
{
    ignoreIds.clear();
    this->set_parameter(rclcpp::Parameter("ignore_fiducials", msg->data));
    handleIgnoreString(msg->data);
}

void camInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    if (haveCamInfo) {
        return;
    }

    if (msg->k != std::array<double, 9>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0})) {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                cameraMatrix.at<double>(i, j) = msg->k[i * 3 + j];
            }
        }

        for (int i = 0; i < 5; i++) {
            distortionCoeffs.at<double>(0, i) = msg->d[i];
        }

        haveCamInfo = true;
        frameId = msg->header.frame_id;
    }
    else {
        RCLCPP_WARN(this->get_logger(), "CameraInfo message has invalid intrinsics, K matrix all zeros");
    }
}

void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (!enable_detections) {
        return; //return without doing anything
    }

    frameNum++;

    cv_bridge::CvImagePtr cv_ptr;

    fiducial_msgs::msg::FiducialTransformArray fta;
    fta.header.stamp = msg->header.stamp;
    fta.header.frame_id = frameId;

    fiducial_msgs::msg::FiducialArray fva;
    fva.header.stamp = msg->header.stamp;
    fva.header.frame_id = frameId;

    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        vector<int> ids;
        vector<vector<Point2f>> corners, rejected;
        vector<Vec3d> rvecs, tvecs;

        aruco::detectMarkers(cv_ptr->image, dictionary, corners, ids, detectorParams);
        if (!ids.empty()) {
            RCLCPP_INFO(this->get_logger(), "Detected %d markers", (int)ids.size());
        }

        for (size_t i = 0; i < ids.size(); i++) {
            if (std::count(ignoreIds.begin(), ignoreIds.end(), ids[i]) != 0) {
                RCLCPP_INFO(this->get_logger(), "Ignoring id %d", ids[i]);
                continue;
            }
            fiducial_msgs::msg::Fiducial fid;
            fid.fiducial_id = ids[i];

            fid.x0 = corners[i][0].x;
            fid.y0 = corners[i][0].y;
            fid.x1 = corners[i][1].x;
            fid.y1 = corners[i][1].y;
            fid.x2 = corners[i][2].x;
            fid.y2 = corners[i][2].y;
            fid.x3 = corners[i][3].x;
            fid.y3 = corners[i][3].y;
            fva.fiducials.push_back(fid);
        }

        vertices_pub->publish(fva);

        if (!ids.empty()) {
            aruco::drawDetectedMarkers(cv_ptr->image, corners, ids);
        }

        if (doPoseEstimation) {
            if (!haveCamInfo) {
                if (frameNum > 5) {
                    RCLCPP_ERROR(this->get_logger(), "No camera intrinsics");
                }
                return;
            }

            vector<double> reprojectionError;
            estimatePoseSingleMarkers(ids, corners, (float)fiducial_len,
                                      cameraMatrix, distortionCoeffs,
                                      rvecs, tvecs,
                                      reprojectionError);

            for (size_t i = 0; i < ids.size(); i++) {
                aruco::drawAxis(cv_ptr->image, cameraMatrix, distortionCoeffs,
                                rvecs[i], tvecs[i], (float)fiducial_len);

                RCLCPP_INFO(this->get_logger(), "Detected id %d T %.2f %.2f %.2f R %.2f %.2f %.2f", ids[i],
                            tvecs[i][0], tvecs[i][1], tvecs[i][2],
                            rvecs[i][0], rvecs[i][1], rvecs[i][2]);

                if (std::count(ignoreIds.begin(), ignoreIds.end(), ids[i]) != 0) {
                    RCLCPP_INFO(this->get_logger(), "Ignoring id %d", ids[i]);
                    continue;
                }

                double angle = norm(rvecs[i]);
                Vec3d axis = rvecs[i] / angle;
                RCLCPP_INFO(this->get_logger(), "angle %f axis %f %f %f",
                            angle, axis[0], axis[1], axis[2]);

                fiducial_msgs::msg::FiducialTransform ft;
                ft.fiducial_id = ids[i];

                ft.transform.translation.x = tvecs[i][0];
                ft.transform.translation.y = tvecs[i][1];
                ft.transform.translation.z = tvecs[i][2];

                tf2::Quaternion q;
                q.setRotation(tf2::Vector3(axis[0], axis[1], axis[2]), angle);

                ft.transform.rotation.w = q.w();
                ft.transform.rotation.x = q.x();
                ft.transform.rotation.y = q.y();
                ft.transform.rotation.z = q.z();

                ft.fiducial_area = calcFiducialArea(corners[i]);
                ft.image_error = reprojectionError[i];

                // Convert image_error (in pixels) to object_error (in meters)
                ft.object_error =
                    (reprojectionError[i] / dist(corners[i][0], corners[i][2])) *
                    (norm(tvecs[i]) / fiducial_len);

                fta.transforms.push_back(ft);
            }
            pose_pub->publish(fta);
        }

        if (publish_images) {
            image_pub.publish(cv_ptr->toImageMsg());
        }
    }
    catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
    catch (cv::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv exception: %s", e.what());
    }
}


void estimatePoseSingleMarkers(const vector<int> &ids,
                               const vector<vector<Point2f>>& corners,
                               float markerLength,
                               const cv::Mat &cameraMatrix,
                               const cv::Mat &distCoeffs,
                               vector<Vec3d>& rvecs, vector<Vec3d>& tvecs,
                               vector<double>& reprojectionError) {

    CV_Assert(markerLength > 0);

    vector<Point3f> markerObjPoints;
    int nMarkers = static_cast<int>(corners.size());
    rvecs.reserve(nMarkers);
    tvecs.reserve(nMarkers);
    reprojectionError.reserve(nMarkers);

    // For each marker, calculate its pose
    for (int i = 0; i < nMarkers; i++) {
        double fiducialSize = markerLength;

        auto it = fiducialLens.find(ids[i]);
        if (it != fiducialLens.end()) {
            fiducialSize = it->second;
        }

        getSingleMarkerObjectPoints(fiducialSize, markerObjPoints);
        cv::solvePnP(markerObjPoints, corners[i], cameraMatrix, distCoeffs,
                     rvecs[i], tvecs[i]);

        reprojectionError[i] =
            getReprojectionError(markerObjPoints, corners[i],
                                 cameraMatrix, distCoeffs,
                                 rvecs[i], tvecs[i]);
    }
}

void handleIgnoreString(const std::string& str) {
    /*
    Ignore fiducials can take a comma-separated list of individual
    fiducial ids or ranges, e.g., "1,4,8,9-12,30-40"
    */
    std::stringstream ss(str);
    std::string element;

    while (std::getline(ss, element, ',')) {
        if (element.empty()) {
            continue;
        }
        std::stringstream range_ss(element);
        std::string range;
        std::vector<std::string> ranges;
        
        while (std::getline(range_ss, range, '-')) {
            ranges.push_back(range);
        }

        if (ranges.size() == 2) {
            int start = std::stoi(ranges[0]);
            int end = std::stoi(ranges[1]);
            RCLCPP_INFO(this->get_logger(), "Ignoring fiducial id range %d to %d", start, end);
            for (int j = start; j <= end; j++) {
                ignoreIds.push_back(j);
            }
        } else if (ranges.size() == 1) {
            int fid = std::stoi(ranges[0]);
            RCLCPP_INFO(this->get_logger(), "Ignoring fiducial id %d", fid);
            ignoreIds.push_back(fid);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Malformed ignore_fiducials: %s", element.c_str());
        }
    }
}

bool enableDetectionsCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                              std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
    enable_detections = req->data;
    if (enable_detections) {
        res->message = "Enabled aruco detections.";
        RCLCPP_INFO(this->get_logger(), "Enabled aruco detections.");
    } else {
        res->message = "Disabled aruco detections.";
        RCLCPP_INFO(this->get_logger(), "Disabled aruco detections.");
    }

    res->success = true;
    return true;
}

FiducialsNode::FiducialsNode() : Node("fiducials_node"), it(this) {
    frameNum = 0;

    // Camera intrinsics
    cameraMatrix = cv::Mat::zeros(3, 3, CV_64F);

    // Distortion coefficients
    distortionCoeffs = cv::Mat::zeros(1, 5, CV_64F);

    haveCamInfo = false;
    enable_detections = true;

    int dicno;

    detectorParams = cv::aruco::DetectorParameters::create();

    // Declare and get parameters
    this->declare_parameter<bool>("publish_images", false);
    this->declare_parameter<double>("fiducial_len", 0.14);
    this->declare_parameter<int>("dictionary", 7);
    this->declare_parameter<bool>("do_pose_estimation", true);
    this->declare_parameter<std::string>("ignore_fiducials", "");
    this->declare_parameter<std::string>("fiducial_len_override", "");

    this->get_parameter("publish_images", publish_images);
    this->get_parameter("fiducial_len", fiducial_len);
    this->get_parameter("dictionary", dicno);
    this->get_parameter("do_pose_estimation", doPoseEstimation);

    std::string str;
    this->get_parameter("ignore_fiducials", str);
    handleIgnoreString(str);

    /*
    Fiducial size can take comma-separated list of size: id or size: range,
    e.g., "200.0: 12, 300.0: 200-300"
    */
    std::vector<std::string> strs;
    this->get_parameter("fiducial_len_override", str);
    std::stringstream ss(str);
    std::string element;

    while (std::getline(ss, element, ',')) {
        if (element.empty()) {
            continue;
        }
        std::vector<std::string> parts;
        std::stringstream part_ss(element);
        std::string part;

        while (std::getline(part_ss, part, ':')) {
            parts.push_back(part);
        }

        if (parts.size() == 2) {
            double len = std::stod(parts[1]);
            std::vector<std::string> range;
            std::stringstream range_ss(parts[0]);
            std::string range_part;

            while (std::getline(range_ss, range_part, '-')) {
                range.push_back(range_part);
            }

            if (range.size() == 2) {
                int start = std::stoi(range[0]);
                int end = std::stoi(range[1]);
                RCLCPP_INFO(this->get_logger(), "Setting fiducial id range %d - %d length to %f",
                            start, end, len);
                for (int j = start; j <= end; j++) {
                    fiducialLens[j] = len;
                }
            } else if (range.size() == 1) {
                int fid = std::stoi(range[0]);
                RCLCPP_INFO(this->get_logger(), "Setting fiducial id %d length to %f", fid, len);
                fiducialLens[fid] = len;
            } else {
                RCLCPP_ERROR(this->get_logger(), "Malformed fiducial_len_override: %s", element.c_str());
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Malformed fiducial_len_override: %s", element.c_str());
        }
    }

    image_pub = it.advertise("/fiducial_images", 1);

    vertices_pub = this->create_publisher<fiducial_msgs::msg::FiducialArray>("fiducial_vertices", 10);

    pose_pub = this->create_publisher<fiducial_msgs::msg::FiducialTransformArray>("fiducial_transforms", 10);

    // Dictionary initialization
    dictionary = cv::aruco::getPredefinedDictionary(dicno);
}
// Dictionary definition (updated)
const cv::Ptr<cv::aruco::Dictionary> COMPETITION_MARKERS = cv::makePtr<cv::aruco::Dictionary>(
    cv::Mat(
        11,
        (7 * 7 + 7) / 8,  // 7 bits x 7 bits + 7 to avoid integer truncation / 8 bits.
        CV_8UC4,
        (uchar *)COMPETITION_MARKERS_7X7),
    7,
    3);

dictionary = COMPETITION_MARKERS;

// Subscriber and Service Initialization (updated for ROS2)
img_sub = it.subscribe("camera", 1, std::bind(&FiducialsNode::imageCallback, this, std::placeholders::_1));

caminfo_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "camera_info", 1, std::bind(&FiducialsNode::camInfoCallback, this, std::placeholders::_1));

ignore_sub = this->create_subscription<std_msgs::msg::String>(
    "ignore_fiducials", 1, std::bind(&FiducialsNode::ignoreCallback, this, std::placeholders::_1));

service_enable_detections = this->create_service<std_srvs::srv::SetBool>(
    "enable_detections", std::bind(&FiducialsNode::enableDetectionsCallback, this, std::placeholders::_1, std::placeholders::_2));

// Parameter Handling
this->declare_parameter<double>("adaptiveThreshConstant", 7.0);
this->declare_parameter<int>("adaptiveThreshWinSizeMax", 53);
this->declare_parameter<int>("adaptiveThreshWinSizeMin", 3);
this->declare_parameter<int>("adaptiveThreshWinSizeStep", 4);
this->declare_parameter<int>("cornerRefinementMaxIterations", 30);
this->declare_parameter<double>("cornerRefinementMinAccuracy", 0.01);
this->declare_parameter<int>("cornerRefinementWinSize", 5);
this->declare_parameter<bool>("doCornerRefinement", true);
this->declare_parameter<bool>("cornerRefinementSubPix", true);
this->declare_parameter<double>("errorCorrectionRate", 0.6);
this->declare_parameter<double>("minCornerDistanceRate", 0.05);
this->declare_parameter<int>("markerBorderBits", 1);
this->declare_parameter<double>("maxErroneousBitsInBorderRate", 0.04);
this->declare_parameter<int>("minDistanceToBorder", 3);
this->declare_parameter<double>("minMarkerDistanceRate", 0.05);
this->declare_parameter<double>("minMarkerPerimeterRate", 0.1);
this->declare_parameter<double>("maxMarkerPerimeterRate", 4.0);
this->declare_parameter<double>("minOtsuStdDev", 5.0);
this->declare_parameter<double>("perspectiveRemoveIgnoredMarginPerCell", 0.13);
this->declare_parameter<int>("perspectiveRemovePixelPerCell", 8);
this->declare_parameter<double>("polygonalApproxAccuracyRate", 0.01);

// Get Parameters
this->get_parameter("adaptiveThreshConstant", detectorParams->adaptiveThreshConstant);
this->get_parameter("adaptiveThreshWinSizeMax", detectorParams->adaptiveThreshWinSizeMax);
this->get_parameter("adaptiveThreshWinSizeMin", detectorParams->adaptiveThreshWinSizeMin);
this->get_parameter("adaptiveThreshWinSizeStep", detectorParams->adaptiveThreshWinSizeStep);
this->get_parameter("cornerRefinementMaxIterations", detectorParams->cornerRefinementMaxIterations);
this->get_parameter("cornerRefinementMinAccuracy", detectorParams->cornerRefinementMinAccuracy);
this->get_parameter("cornerRefinementWinSize", detectorParams->cornerRefinementWinSize);

bool doCornerRefinement;
this->get_parameter("doCornerRefinement", doCornerRefinement);
if (doCornerRefinement) {
    bool cornerRefinementSubPix;
    this->get_parameter("cornerRefinementSubPix", cornerRefinementSubPix);
    if (cornerRefinementSubPix) {
        detectorParams->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    } else {
        detectorParams->cornerRefinementMethod = cv::aruco::CORNER_REFINE_CONTOUR;
    }
} else {
    detectorParams->cornerRefinementMethod = cv::aruco::CORNER_REFINE_NONE;
}

this->get_parameter("errorCorrectionRate", detectorParams->errorCorrectionRate);
this->get_parameter("minCornerDistanceRate", detectorParams->minCornerDistanceRate);
this->get_parameter("markerBorderBits", detectorParams->markerBorderBits);
this->get_parameter("maxErroneousBitsInBorderRate", detectorParams->maxErroneousBitsInBorderRate);
this->get_parameter("minDistanceToBorder", detectorParams->minDistanceToBorder);
this->get_parameter("minMarkerDistanceRate", detectorParams->minMarkerDistanceRate);
this->get_parameter("minMarkerPerimeterRate", detectorParams->minMarkerPerimeterRate);
this->get_parameter("maxMarkerPerimeterRate", detectorParams->maxMarkerPerimeterRate);
this->get_parameter("minOtsuStdDev", detectorParams->minOtsuStdDev);
this->get_parameter("perspectiveRemoveIgnoredMarginPerCell", detectorParams->perspectiveRemoveIgnoredMarginPerCell);
this->get_parameter("perspectiveRemovePixelPerCell", detectorParams->perspectiveRemovePixelPerCell);
this->get_parameter("polygonalApproxAccuracyRate", detectorParams->polygonalApproxAccuracyRate);

RCLCPP_INFO(this->get_logger(), "Aruco detection ready");

#endif  // FIDUCIALS_NODE_H