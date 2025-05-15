#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "std_srvs/srv/set_bool.hpp"
#include "rover_msgs/msg/b_box_stats.hpp"
#include "rover_msgs/msg/b_box_stats_array.hpp"

using namespace std::chrono_literals;

class PointCloudProcessor : public rclcpp::Node
{
public:
  PointCloudProcessor()
  : Node("pcl_processor"), latest_cloud_(new pcl::PointCloud<pcl::PointXYZRGB>)
  {

    // Shared X and Z
    this->declare_parameter<double>("bbox_min_x", -1.0, rcl_interfaces::msg::ParameterDescriptor().set__description("Bounding box minimum X coordinate (meters)"));
    this->declare_parameter<double>("bbox_max_x",  1.0, rcl_interfaces::msg::ParameterDescriptor().set__description("Bounding box maximum X coordinate (meters)"));
    this->declare_parameter<double>("bbox_min_y", -1.0, rcl_interfaces::msg::ParameterDescriptor().set__description("Bounding box minimum Y coordinate (meters)"));
    this->declare_parameter<double>("bbox_max_y",  1.0, rcl_interfaces::msg::ParameterDescriptor().set__description("Bounding box maximum Y coordinate (meters)"));
    this->declare_parameter<double>("bbox_min_z", -1.0, rcl_interfaces::msg::ParameterDescriptor().set__description("Bounding box minimum Z coordinate (meters)"));
    this->declare_parameter<double>("bbox_max_z",  1.0, rcl_interfaces::msg::ParameterDescriptor().set__description("Bounding box maximum Z coordinate (meters)"));
    // Left Y
    this->declare_parameter<double>("bbox_max_y_left", 1.0, rcl_interfaces::msg::ParameterDescriptor().set__description("Bounding box maximum Y for left region (meters)"));
    // Right Y
    this->declare_parameter<double>("bbox_min_y_right", -1.0, rcl_interfaces::msg::ParameterDescriptor().set__description("Bounding box minimum Y for right region (meters)"));
    // Publish cropped clouds and bbox markers
    this->declare_parameter<bool>("verbose_publishing", false, rcl_interfaces::msg::ParameterDescriptor().set__description("If true, publish cropped clouds and bbox markers."));
    verbose_publishing_ = this->get_parameter("verbose_publishing").as_bool();



    // Get parameters
    bbox_min_x_ = this->get_parameter("bbox_min_x").as_double();
    bbox_min_y_ = this->get_parameter("bbox_min_y").as_double();
    bbox_min_z_ = this->get_parameter("bbox_min_z").as_double();
    bbox_max_x_ = this->get_parameter("bbox_max_x").as_double();
    bbox_max_y_ = this->get_parameter("bbox_max_y").as_double();
    bbox_max_z_ = this->get_parameter("bbox_max_z").as_double();
    bbox_max_y_left_  = this->get_parameter("bbox_max_y_left").as_double();
    bbox_min_y_right_ = this->get_parameter("bbox_min_y_right").as_double();


    // Subscriber for input cloud
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/zed/zed_node/point_cloud/cloud_registered", 10,
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        pcl::fromROSMsg(*msg, *latest_cloud_);
      });

    // Publisher for cropped cloud
    cropped_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("cropped_cloud", 10);
    cropped_left_pub_  = this->create_publisher<sensor_msgs::msg::PointCloud2>("cropped_left", 10);
    cropped_right_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cropped_right", 10);


    // Publisher for bounding box visualization
    bbox_pub_ = create_publisher<visualization_msgs::msg::Marker>("bbox_marker", 10);

    // Hazards publisher
    bbox_stats_pub_ = this->create_publisher<rover_msgs::msg::BBoxStatsArray>("hazard_stats", 10);


    // Processing timer
    timer_ = create_wall_timer(
      100ms, [this]() { process_and_publish(); });
    
    // Configure crop box parameters (adjust these values as needed)
    // Set crop box parameters
    crop_box_.setMin(Eigen::Vector4f(bbox_min_x_, bbox_min_y_, bbox_min_z_, 1.0));
    crop_box_.setMax(Eigen::Vector4f(bbox_max_x_, bbox_max_y_, bbox_max_z_, 1.0));
    

    // TODO: setup for new parameters
    param_callback_handle_ = this->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &params) {
          for (const auto &param : params) {
            if (param.get_name() == "bbox_min_x") bbox_min_x_ = param.as_double();
            if (param.get_name() == "bbox_min_y") bbox_min_y_ = param.as_double();
            if (param.get_name() == "bbox_min_z") bbox_min_z_ = param.as_double();
            if (param.get_name() == "bbox_max_x") bbox_max_x_ = param.as_double();
            if (param.get_name() == "bbox_max_y") bbox_max_y_ = param.as_double();
            if (param.get_name() == "bbox_max_z") bbox_max_z_ = param.as_double();
          }
          crop_box_.setMin(Eigen::Vector4f(bbox_min_x_, bbox_min_y_, bbox_min_z_, 1.0));
          crop_box_.setMax(Eigen::Vector4f(bbox_max_x_, bbox_max_y_, bbox_max_z_, 1.0));
          return rcl_interfaces::msg::SetParametersResult().set__successful(true);
        }
      );

      enabled_ = false;  // default state

      enable_service_ = this->create_service<std_srvs::srv::SetBool>(
          "/hazard_detector/enable",
          std::bind(&PointCloudProcessor::enable_callback, this,
                    std::placeholders::_1, std::placeholders::_2));
      
      RCLCPP_INFO(this->get_logger(), "Hazard_detector initialized");
      RCLCPP_WARN(this->get_logger(), "Hazard detection is DISABLED. Use the /hazard_detector/enable service to enable it.");
  }

private:
void enable_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    enabled_ = request->data;
    RCLCPP_INFO(this->get_logger(), "Hazard Detection: %s", enabled_ ? "ENABLED" : "DISABLED");
    response->success = true;
    response->message = std::string("Hazard Detection: ") + (enabled_ ? "ENABLED" : "DISABLED");
} 

rover_msgs::msg::BBoxStats make_stats(const pcl::PointCloud<pcl::PointXYZRGB>& cloud, const std::string& region)
{
    rover_msgs::msg::BBoxStats stats;
    stats.count = static_cast<int32_t>(cloud.size());
    stats.region = region;
    if (!cloud.empty()) {
        auto min_x_it = std::min_element(cloud.points.begin(), cloud.points.end(),
                                         [](const auto& a, const auto& b) { return a.x < b.x; });
        stats.min_x = min_x_it->x;
        stats.min_z = min_x_it->z;
    } else {
        stats.min_x = std::numeric_limits<float>::quiet_NaN();
        stats.min_z = std::numeric_limits<float>::quiet_NaN();
    }
    return stats;
}


void process_and_publish()
  {
    if (!enabled_) return; // Skip processing if not enabled
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    
    if (latest_cloud_->empty()) return;

    // Left
    pcl::CropBox<pcl::PointXYZRGB> crop_box_left;
    crop_box_left.setMin(Eigen::Vector4f(bbox_min_x_, bbox_max_y_, bbox_min_z_, 1.0));
    crop_box_left.setMax(Eigen::Vector4f(bbox_max_x_, bbox_max_y_left_, bbox_max_z_, 1.0));
    crop_box_left.setInputCloud(latest_cloud_);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cropped_left(new pcl::PointCloud<pcl::PointXYZRGB>);
    crop_box_left.filter(*cropped_left);

    // Right
    pcl::CropBox<pcl::PointXYZRGB> crop_box_right;
    crop_box_right.setMin(Eigen::Vector4f(bbox_min_x_, bbox_min_y_right_, bbox_min_z_, 1.0));
    crop_box_right.setMax(Eigen::Vector4f(bbox_max_x_, bbox_min_y_, bbox_max_z_, 1.0));
    crop_box_right.setInputCloud(latest_cloud_);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cropped_right(new pcl::PointCloud<pcl::PointXYZRGB>);
    crop_box_right.filter(*cropped_right);

    
    // Crop the point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    crop_box_.setInputCloud(latest_cloud_);
    crop_box_.filter(*cropped_cloud);
    
    // Publish cropped cloud
    if (verbose_publishing_) {
      // Publish
      sensor_msgs::msg::PointCloud2 left_msg, right_msg;
      pcl::toROSMsg(*cropped_left, left_msg);
      pcl::toROSMsg(*cropped_right, right_msg);
      left_msg.header.frame_id = right_msg.header.frame_id = "zed_camera_link";
      left_msg.header.stamp = right_msg.header.stamp = this->now();
  
      cropped_left_pub_->publish(left_msg);
      cropped_right_pub_->publish(right_msg);

      
      RCLCPP_INFO(this->get_logger(), "Publishing cropped cloud");
      sensor_msgs::msg::PointCloud2 output_msg;
      pcl::toROSMsg(*cropped_cloud, output_msg);
      output_msg.header = std_msgs::msg::Header();
      output_msg.header.frame_id = "zed_camera_link"; // Update with your frame
      output_msg.header.stamp = now();
      cropped_pub_->publish(output_msg);
      
      
      // Create and publish bounding box marker
      auto marker_straight = create_bbox_marker(bbox_min_x_, bbox_max_x_, bbox_min_y_, bbox_max_y_, bbox_min_z_, bbox_max_z_, 0, 1.0, 0.0, 0.0);
      auto marker_left = create_bbox_marker(bbox_min_x_, bbox_max_x_, bbox_max_y_, bbox_max_y_left_, bbox_min_z_, bbox_max_z_, 1, 0.0, 1.0, 0.0);
      auto marker_right = create_bbox_marker(bbox_min_x_, bbox_max_x_, bbox_min_y_right_, bbox_min_y_, bbox_min_z_, bbox_max_z_, 2, 0.0, 0.0, 1.0);
      bbox_pub_->publish(marker_straight);
      bbox_pub_->publish(marker_left);
      bbox_pub_->publish(marker_right);
    }

    

    rover_msgs::msg::BBoxStatsArray stats_array;
    // IMPORTANT FOR NOW MAKE THE STRAIGHT BOX THE FIRST, RIGHT THE SECOND AND LEFT THE LAST
    stats_array.boxes.push_back(make_stats(*cropped_cloud, "straight"));
    stats_array.boxes.push_back(make_stats(*cropped_right, "right"));
    stats_array.boxes.push_back(make_stats(*cropped_left, "left"));
    stats_array.header.frame_id = "zed_camera_link";
    stats_array.header.stamp = this->now();
    bbox_stats_pub_->publish(stats_array);

    // Clear the latest cloud to avoid reprocessing
    latest_cloud_->clear();
  }

    visualization_msgs::msg::Marker create_bbox_marker(
        double min_x, double max_x,
        double min_y, double max_y,
        double min_z, double max_z,
        int id, float r, float g, float b)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "zed_camera_link";
        marker.header.stamp = this->now();
        marker.ns = "bbox";
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.scale.x = max_x - min_x;
        marker.scale.y = max_y - min_y;
        marker.scale.z = max_z - min_z;

        marker.pose.position.x = (max_x + min_x) / 2.0;
        marker.pose.position.y = (max_y + min_y) / 2.0;
        marker.pose.position.z = (max_z + min_z) / 2.0;
        marker.pose.orientation.w = 1.0;

        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 0.3f;
        marker.lifetime = rclcpp::Duration::from_seconds(0.2);
        return marker;
    }


  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cropped_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cropped_right_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cropped_left_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr bbox_pub_;
  rclcpp::Publisher<rover_msgs::msg::BBoxStatsArray>::SharedPtr bbox_stats_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Store parameters as member variables
  double bbox_min_x_, bbox_min_y_, bbox_min_z_;
  double bbox_max_x_, bbox_max_y_, bbox_max_z_;
  double bbox_max_y_left_, bbox_min_y_right_;
  
  pcl::CropBox<pcl::PointXYZRGB> crop_box_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr latest_cloud_;
  std::mutex cloud_mutex_;
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // Service
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;
  bool enabled_;
  bool verbose_publishing_ = false;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudProcessor>());
  rclcpp::shutdown();
  return 0;
}
