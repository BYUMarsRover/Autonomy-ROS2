// FiducialsNode.cpp
#include "FiducialsNode.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>

#include <rover_msgs/msg/fiducial.hpp>
#include <rover_msgs/msg/fiducial_array.hpp>
#include <rover_msgs/msg/fiducial_transform.hpp>
#include <rover_msgs/msg/fiducial_transform_array.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <image_transport/image_transport.hpp>

// Constructor implementation
FiducialsNode::FiducialsNode()
    : Node("fiducials_node"),
      image_transport_(std::enable_shared_from_this<FiducialsNode>::shared_from_this()),
      have_cam_info_(false),
      frame_num_(0),
      enable_detections_(true) {
    // Declare and get parameters
    this->declare_parameter<bool>("publish_images", false);
    this->declare_parameter<double>("fiducial_len", 0.14);
    this->declare_parameter<bool>("do_pose_estimation", true);
    this->declare_parameter<std::string>("dictionary", "DICT_6X6_250");
    this->declare_parameter<std::string>("ignore_fiducials", "");
    this->declare_parameter<std::string>("fiducial_len_override", "");
    this->declare_parameter<std::string>("frame_id", "camera_frame");

    this->get_parameter("publish_images", publish_images_);
    this->get_parameter("fiducial_len", fiducial_len_);
    this->get_parameter("do_pose_estimation", do_pose_estimation_);
    this->get_parameter("frame_id", frame_id_);

    // Initialize publishers
    vertices_pub_ = this->create_publisher<rover_msgs::msg::FiducialArray>("fiducial_vertices", 10);
    pose_pub_ = this->create_publisher<rover_msgs::msg::FiducialTransformArray>("fiducial_transforms", 10);
    vertices_pub_ = this->create_publisher<rover_msgs::msg::FiducialArray>("fiducial_vertices", 10);
    pose_pub_ = this->create_publisher<rover_msgs::msg::FiducialTransformArray>("fiducial_transforms", 10);
    if (publish_images_) {
        image_pub_ = image_transport_.advertise("fiducial_images", 10);
    }

    // Initialize subscribers
    caminfo_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "camera_info", 10, std::bind(&FiducialsNode::camInfoCallback, this, std::placeholders::_1));
    ignore_sub_ = this->create_subscription<std_msgs::msg::String>(
        "ignore_fiducials", 10, std::bind(&FiducialsNode::ignoreCallback, this, std::placeholders::_1));
    image_sub_ = image_transport_.subscribe(
        "image_raw", 10, std::bind(&FiducialsNode::imageCallback, this, std::placeholders::_1));

    // Initialize service
    enable_detections_srv_ = this->create_service<std_srvs::srv::SetBool>(
        "enable_detections", std::bind(&FiducialsNode::enableDetectionsCallback, this,
                                       std::placeholders::_1, std::placeholders::_2));

    // Initialize parameter callback
    parameter_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&FiducialsNode::parameterCallback, this, std::placeholders::_1));

    // Initialize ArUco detector parameters
    detector_params_ = cv::aruco::DetectorParameters::create();

    // Set up the ArUco dictionary
    std::string dictionary_name;
    this->get_parameter("dictionary", dictionary_name);
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

    // Handle ignored fiducials
    std::string ignore_fiducials;
    this->get_parameter("ignore_fiducials", ignore_fiducials);
    handleIgnoreString(ignore_fiducials);

    // Handle fiducial length overrides
    std::string fiducial_len_override;
    this->get_parameter("fiducial_len_override", fiducial_len_override);
    parseFiducialLenOverride(fiducial_len_override);

    RCLCPP_INFO(this->get_logger(), "FiducialsNode initialized");
}


// Image callback
void FiducialsNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    if (!enable_detections_) {
        return;
    }

    frame_num_++;

    cv_bridge::CvImagePtr cv_ptr;

    auto fta = rover_msgs::msg::FiducialTransformArray();
    fta.header.stamp = msg->header.stamp;
    fta.header.frame_id = frame_id_;

    auto fva = rover_msgs::msg::FiducialArray();
    fva.header.stamp = msg->header.stamp;
    fva.header.frame_id = frame_id_;

    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners, rejected;
        std::vector<cv::Vec3d> rvecs, tvecs;

        cv::aruco::detectMarkers(cv_ptr->image, dictionary_, corners, ids, detector_params_);
        if (!ids.empty()) {
            RCLCPP_INFO(this->get_logger(), "Detected %d markers", static_cast<int>(ids.size()));
        }

        for (size_t i = 0; i < ids.size(); i++) {
            if (std::find(ignore_ids_.begin(), ignore_ids_.end(), ids[i]) != ignore_ids_.end()) {
                RCLCPP_INFO(this->get_logger(), "Ignoring id %d", ids[i]);
                continue;
            }
            auto fid = rover_msgs::msg::Fiducial();
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

        vertices_pub_->publish(fva);

        if (!ids.empty()) {
            cv::aruco::drawDetectedMarkers(cv_ptr->image, corners, ids);
        }

        if (do_pose_estimation_) {
            if (!have_cam_info_) {
                if (frame_num_ > 5) {
                    RCLCPP_ERROR(this->get_logger(), "No camera intrinsics");
                }
                return;
            }

            std::vector<double> reprojection_errors;
            estimatePoseSingleMarkers(ids, corners, static_cast<float>(fiducial_len_),
                                      camera_matrix_, distortion_coeffs_,
                                      rvecs, tvecs, reprojection_errors);

            for (size_t i = 0; i < ids.size(); i++) {
                cv::aruco::drawAxis(cv_ptr->image, camera_matrix_, distortion_coeffs_,
                                    rvecs[i], tvecs[i], static_cast<float>(fiducial_len_));

                RCLCPP_INFO(this->get_logger(),
                            "Detected id %d T [%.2f %.2f %.2f] R [%.2f %.2f %.2f]",
                            ids[i], tvecs[i][0], tvecs[i][1], tvecs[i][2],
                            rvecs[i][0], rvecs[i][1], rvecs[i][2]);

                if (std::find(ignore_ids_.begin(), ignore_ids_.end(), ids[i]) != ignore_ids_.end()) {
                    RCLCPP_INFO(this->get_logger(), "Ignoring id %d", ids[i]);
                    continue;
                }

                double angle = cv::norm(rvecs[i]);
                cv::Vec3d axis = rvecs[i] / angle;
                RCLCPP_INFO(this->get_logger(), "Angle %f Axis [%f %f %f]",
                            angle, axis[0], axis[1], axis[2]);

                auto ft = rover_msgs::msg::FiducialTransform();
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

                ft.fiducial_area = calculateFiducialArea(corners[i]);
                ft.image_error = reprojection_errors[i];

                // Convert image_error (in pixels) to object_error (in meters)
                ft.object_error =
                    (reprojection_errors[i] / cv::norm(corners[i][0] - corners[i][2])) *
                    (cv::norm(tvecs[i]) / fiducial_len_);

                fta.transforms.push_back(ft);
            }
            pose_pub_->publish(fta);
        }

        if (publish_images_ && image_pub_.getNumSubscribers() > 0) {
            image_pub_.publish(cv_ptr->toImageMsg());
        }
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    } catch (cv::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "OpenCV exception: %s", e.what());
    }
}

// Camera info callback
void FiducialsNode::camInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
    if (have_cam_info_) {
        return;
    }

    if (std::all_of(msg->k.begin(), msg->k.end(), [](double k) { return k == 0.0; })) {
        RCLCPP_WARN(this->get_logger(), "CameraInfo message has invalid intrinsics, K matrix all zeros");
        return;
    }

    camera_matrix_ = cv::Mat(3, 3, CV_64F, (void*)msg->k.data()).clone();
    distortion_coeffs_ = cv::Mat(msg->d.size(), 1, CV_64F, (void*)msg->d.data()).clone();

    have_cam_info_ = true;
    frame_id_ = msg->header.frame_id;
}

// Ignore fiducials callback
void FiducialsNode::ignoreCallback(const std_msgs::msg::String::SharedPtr msg) {
    ignore_ids_.clear();
    this->set_parameter(rclcpp::Parameter("ignore_fiducials", msg->data));
    handleIgnoreString(msg->data);
}

// Enable detections service callback
void FiducialsNode::enableDetectionsCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    enable_detections_ = request->data;
    response->success = true;
    if (enable_detections_) {
        response->message = "ArUco detections have been enabled.";
        RCLCPP_INFO(this->get_logger(), "ArUco detections enabled.");
    } else {
        response->message = "ArUco detections have been disabled.";
        RCLCPP_INFO(this->get_logger(), "ArUco detections disabled.");
    }
}

// Parameter callback
rcl_interfaces::msg::SetParametersResult FiducialsNode::parameterCallback(
    const std::vector<rclcpp::Parameter>& params) {
    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = true;
    for (const auto& param : params) {
        if (param.get_name() == "fiducial_len") {
            fiducial_len_ = param.as_double();
        } else if (param.get_name() == "publish_images") {
            publish_images_ = param.as_bool();
        } else if (param.get_name() == "enable_detections") {
            enable_detections_ = param.as_bool();
        }
        // Update other parameters as needed
    }
    return result;
}

// Helper functions

void FiducialsNode::handleIgnoreString(const std::string& str) {
    /*
    ignore_fiducials can take comma-separated list of individual
    fiducial ids or ranges, e.g., "1,4,8,9-12,30-40"
    */
    ignore_ids_.clear();
    std::stringstream ss(str);
    std::string item;
    while (std::getline(ss, item, ',')) {
        if (item.empty()) {
            continue;
        }
        size_t dash_pos = item.find('-');
        if (dash_pos != std::string::npos) {
            int start = std::stoi(item.substr(0, dash_pos));
            int end = std::stoi(item.substr(dash_pos + 1));
            RCLCPP_INFO(this->get_logger(), "Ignoring fiducial id range %d to %d", start, end);
            for (int j = start; j <= end; j++) {
                ignore_ids_.push_back(j);
            }
        } else {
            int fid = std::stoi(item);
            RCLCPP_INFO(this->get_logger(), "Ignoring fiducial id %d", fid);
            ignore_ids_.push_back(fid);
        }
    }
}

void FiducialsNode::parseFiducialLenOverride(const std::string& str) {
    /*
    fiducial_len_override can take comma-separated list of id:length pairs, e.g., "1:0.1,2-4:0.2"
    */
    std::stringstream ss(str);
    std::string item;
    while (std::getline(ss, item, ',')) {
        if (item.empty()) {
            continue;
        }
        size_t colon_pos = item.find(':');
        if (colon_pos != std::string::npos) {
            double len = std::stod(item.substr(colon_pos + 1));
            std::string ids_str = item.substr(0, colon_pos);
            size_t dash_pos = ids_str.find('-');
            if (dash_pos != std::string::npos) {
                int start = std::stoi(ids_str.substr(0, dash_pos));
                int end = std::stoi(ids_str.substr(dash_pos + 1));
                RCLCPP_INFO(this->get_logger(), "Setting fiducial id range %d - %d length to %f", start, end, len);
                for (int j = start; j <= end; j++) {
                    fiducial_lens_[j] = len;
                }
            } else {
                int fid = std::stoi(ids_str);
                RCLCPP_INFO(this->get_logger(), "Setting fiducial id %d length to %f", fid, len);
                fiducial_lens_[fid] = len;
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Malformed fiducial_len_override: %s", item.c_str());
        }
    }
}

void FiducialsNode::estimatePoseSingleMarkers(
    const std::vector<int>& ids,
    const std::vector<std::vector<cv::Point2f>>& corners,
    float marker_length,
    const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs,
    std::vector<cv::Vec3d>& rvecs,
    std::vector<cv::Vec3d>& tvecs,
    std::vector<double>& reprojection_errors) {

    CV_Assert(marker_length > 0);

    size_t n_markers = corners.size();
    rvecs.resize(n_markers);
    tvecs.resize(n_markers);
    reprojection_errors.resize(n_markers);

    // For each marker, calculate its pose
    for (size_t i = 0; i < n_markers; i++) {
        double fiducial_size = marker_length;

        auto it = fiducial_lens_.find(ids[i]);
        if (it != fiducial_lens_.end()) {
            fiducial_size = it->second;
        }

        std::vector<cv::Point3f> marker_obj_points;
        getSingleMarkerObjectPoints(fiducial_size, marker_obj_points);

        cv::solvePnP(marker_obj_points, corners[i], camera_matrix, dist_coeffs,
                     rvecs[i], tvecs[i]);

        reprojection_errors[i] = getReprojectionError(marker_obj_points, corners[i],
                                                      camera_matrix, dist_coeffs,
                                                      rvecs[i], tvecs[i]);
    }
}

// Utility functions
void FiducialsNode::getSingleMarkerObjectPoints(float marker_length, std::vector<cv::Point3f>& obj_points) {
    CV_Assert(marker_length > 0);

    // Set coordinate system in the middle of the marker, with Z pointing out
    obj_points.clear();
    obj_points.push_back(cv::Point3f(-marker_length / 2.f, marker_length / 2.f, 0));
    obj_points.push_back(cv::Point3f(marker_length / 2.f, marker_length / 2.f, 0));
    obj_points.push_back(cv::Point3f(marker_length / 2.f, -marker_length / 2.f, 0));
    obj_points.push_back(cv::Point3f(-marker_length / 2.f, -marker_length / 2.f, 0));
}

// Calculate Euclidean distance between two points
double FiducialsNode::calculateDistance(const cv::Point2f& p1, const cv::Point2f& p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return std::sqrt(dx * dx + dy * dy);
}

// Compute area of a fiducial using Heron's formula
double FiducialsNode::calculateFiducialArea(const std::vector<cv::Point2f>& points) {
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
double FiducialsNode::getReprojectionError(
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
