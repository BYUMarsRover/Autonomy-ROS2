#ifndef FIDUCIALS_NODE_H  // Include guard to prevent multiple inclusion
#define FIDUCIALS_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include <rover_msgs/msg/fiducial_array.hpp>
#include <rover_msgs/msg/fiducial_transform_array.hpp>

#include <map>
#include <string>
#include <vector>

// JM added: public std::enable_shared_from_this<FiducialsNode> 
class FiducialsNode : public rclcpp::Node, public std::enable_shared_from_this<FiducialsNode> {
public:
    // Constructor
    explicit FiducialsNode();

private:
    // Publishers
    rclcpp::Publisher<rover_msgs::msg::FiducialArray>::SharedPtr vertices_pub_;
    rclcpp::Publisher<rover_msgs::msg::FiducialTransformArray>::SharedPtr pose_pub_;
    image_transport::Publisher image_pub;

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr caminfo_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ignore_sub_;
    image_transport::Subscriber image_sub_;

    // Service server
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_detections_srv_;

    // Parameters
    bool publish_images_;
    bool enable_detections_;
    bool do_pose_estimation_;
    bool have_cam_info_;

    int frame_num_;
    double fiducial_len_;

    // Detector parameters
    double adaptive_thresh_constant_;
    int adaptive_thresh_win_size_min_;
    int adaptive_thresh_win_size_max_;
    int adaptive_thresh_win_size_step_;
    int corner_refinement_max_iterations_;
    double corner_refinement_min_accuracy_;
    int corner_refinement_win_size_;
    bool do_corner_refinement_;
    bool corner_refinement_subpix_;
    double error_correction_rate_;
    double min_corner_distance_rate_;
    int marker_border_bits_;
    double max_erroneous_bits_in_border_rate_;
    double min_distance_to_border_;
    double min_marker_distance_rate_;
    double min_marker_perimeter_rate_;
    double max_marker_perimeter_rate_;
    double min_otsu_std_dev_;
    double perspective_remove_ignored_margin_per_cell_;
    int perspective_remove_pixel_per_cell_;
    double polygonal_approx_accuracy_rate_;

    // Camera parameters
    cv::Mat camera_matrix_;
    cv::Mat distortion_coeffs_;

    std::string frame_id_;
    std::vector<int> ignore_ids_;
    std::map<int, double> fiducial_lens_;

    image_transport::ImageTransport image_transport_;

    // OpenCV and ArUco
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;

    // Member functions
    void handleIgnoreString(const std::string& str);
    void estimatePoseSingleMarkers(
        const std::vector<int>& ids,
        const std::vector<std::vector<cv::Point2f>>& corners,
        float marker_length,
        const cv::Mat& camera_matrix,
        const cv::Mat& dist_coeffs,
        std::vector<cv::Vec3d>& rvecs,
        std::vector<cv::Vec3d>& tvecs,
        std::vector<double>& reprojection_errors);
    
    double calculateDistance(const cv::Point2f& p1, const cv::Point2f& p2);
    double calculateFiducialArea(const std::vector<cv::Point2f>& points);
    double getReprojectionError(
        const std::vector<cv::Point3f>& object_points,
        const std::vector<cv::Point2f>& image_points,
        const cv::Mat& camera_matrix,
        const cv::Mat& dist_coeffs,
        const cv::Vec3d& rvec,
        const cv::Vec3d& tvec);
    void ignoreCallback(const std_msgs::msg::String::SharedPtr msg);
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
    void camInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);


    rcl_interfaces::msg::SetParametersResult parameterCallback(
        const std::vector<rclcpp::Parameter>& params);
};

#endif  // FIDUCIALS_NODE_H
