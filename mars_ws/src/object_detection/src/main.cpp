#include <iostream>
#include <chrono>
#include <cmath>
#include <memory>
#include "cuda_utils.h"
#include "logging.h"
#include "utils.h"

#include "yolo.hpp"
#include "sl_tools.h"

#include <sl/Camera.hpp>
#include <NvInfer.h>
#include <cv_bridge/cv_bridge.h>

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/Image.hpp>
#include <sensor_msgs/Imu.hpp>
#include <sensor_msgs/MagneticField.hpp>
#include <sensor_msgs/distortion_models.hpp>
#include <zed_msgs/ObjectDetection.hpp>
#include <zed_msgs/ObjectDetections.hpp>



using namespace nvinfer1;
#define NMS_THRESH 0.4
#define CONF_THRESH 0.3

#ifndef DEG2RAD
#define DEG2RAD 0.017453293
#define RAD2DEG 57.295777937
#endif

std::string mag_frame_id = "zed2i_mag_link";
std::string imu_frame_id = "zed2i_imu_link";

void print(std::string msg_prefix, sl::ERROR_CODE err_code, std::string msg_suffix) {
    std::cout << "[Sample] ";
    if (err_code != sl::ERROR_CODE::SUCCESS)
        std::cout << "[Error] ";
    std::cout << msg_prefix << " ";
    if (err_code != sl::ERROR_CODE::SUCCESS) {
        std::cout << " | " << toString(err_code) << " : ";
        std::cout << toVerbose(err_code);
    }
    if (!msg_suffix.empty())
        std::cout << " " << msg_suffix;
    std::cout << std::endl;
}

cv::Rect get_rect(BBox box) {
    return cv::Rect(round(box.x1), round(box.y1), round(box.x2 - box.x1), round(box.y2 - box.y1));
}

std::vector<sl::uint2> cvt(const BBox &bbox_in) {
    std::vector<sl::uint2> bbox_out(4);
    bbox_out[0] = sl::uint2(bbox_in.x1, bbox_in.y1);
    bbox_out[1] = sl::uint2(bbox_in.x2, bbox_in.y1);
    bbox_out[2] = sl::uint2(bbox_in.x2, bbox_in.y2);
    bbox_out[3] = sl::uint2(bbox_in.x1, bbox_in.y2);
    return bbox_out;
}

void publish_detections(
    Yolo& detector,
    sl::Mat& left_sl,
    cv::Mat& left_cv,
    sl::ObjectDetectionRuntimeParameters& objectTracker_parameters_rt,
    sl::Objects& objects,
    sl::Resolution& display_resolution,
    sl::Camera& zed,
    ros::Publisher object_detection_pub,
    image_transport::Publisher detection_annotation) {

    if (object_detection_pub.getNumSubscribers() == 0 && detection_annotation.getNumSubscribers() == 0) {
        return;
    }
    // Running inference
    auto detections = detector.run(left_sl, display_resolution.height, display_resolution.width, CONF_THRESH); //STUPID PROBLEM

    // Get image for display
    left_cv = slMat2cvMat(left_sl);

    // Preparing for ZED SDK ingesting
    std::vector<sl::CustomBoxObjectData> objects_in;
    for (auto &it : detections) {
        sl::CustomBoxObjectData tmp;
        // Fill the detections into the correct format
        tmp.unique_object_id = sl::generate_unique_id();
        tmp.probability = it.prob;
        tmp.label = (int) it.label;
        tmp.bounding_box_2d = cvt(it.box);
        tmp.is_grounded = ((int) it.label == 0); // Only the first class (person) is grounded, that is moving on the floor plane
        // others are tracked in full 3D space
        objects_in.push_back(tmp);
    }
    // Send the custom detected boxes to the ZED
    zed.ingestCustomBoxObjects(objects_in);

    if (detection_annotation.getNumSubscribers() > 0) {
        for (size_t j = 0; j < detections.size(); j++) {
            cv::Rect r = get_rect(detections[j].box);
            cv::rectangle(left_cv, r, cv::Scalar(0x27, 0xC1, 0x36), 2);
            cv::putText(left_cv, std::to_string((int) detections[j].label), cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
        }

        std_msgs::Header annotated_img_header;
        cv_bridge::CvImage annotated_img(annotated_img_header, sensor_msgs::image_encodings::TYPE_8UC4, left_cv);
        detection_annotation.publish(annotated_img.toImageMsg());
    }

    if (object_detection_pub.getNumSubscribers() > 0) {
        // Retrieve the tracked objects, with 2D and 3D attributes
        zed.retrieveObjects(objects, objectTracker_parameters_rt);
        if (objects.object_list.size() > 0) {
            zed_msgs::ObjectDetections msg;
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = "object_detections";
            for (auto& object : objects.object_list) {
                zed_msgs::ObjectDetection detection;
                detection.id = object.id;
                detection.label = object.raw_label;
                detection.x = object.position[0];
                detection.y = object.position[1];
                detection.z = object.position[2];
                detection.confidence = object.confidence;
                msg.objects.push_back(detection);
            }
            object_detection_pub.publish(msg);
        }
    }
}



class ObjectDetectionNode : public rclcpp::Node {
public:
    ObjectDetectionNode() : Node("object_detection") {
        // Publishers
        object_detection_pub_ = this->create_publisher<zed_interfaces::msg::ObjectDetections>("/object_detection", 10);
        detection_annotation_ = image_transport::ImageTransport(this).advertise("/object_detection/annotated", 1);

        // ZED and YOLO initialization (not shown, see original code)
        zed_.open(init_parameters_);
        detector_.init(engine_name_);
        
        // Timer for the detection loop
        timer_ = this->create_wall_timer(16ms, std::bind(&ObjectDetectionNode::processFrame, this));
    }

private:
    void processFrame() {
        // Grab image from ZED and process detections
        if (zed_.grab() == sl::ERROR_CODE::SUCCESS) {
            zed_.retrieveImage(left_sl_, sl::VIEW::LEFT);
            publishDetections();
        }
    }

    void publishDetections() {
        // Running inference
        auto detections = detector_.run(left_sl_, display_resolution_.height, display_resolution_.width, CONF_THRESH);

        // Publish detections if subscribers are present
        if (object_detection_pub_->get_subscription_count() > 0) {
            zed_.retrieveObjects(objects_, object_tracker_params_rt_);
            if (!objects_.object_list.empty()) {
                zed_interfaces::msg::ObjectDetections msg;
                msg.header.stamp = now();
                msg.header.frame_id = "object_detections";
                
                for (const auto& object : objects_.object_list) {
                    zed_interfaces::msg::ObjectDetection detection;
                    detection.id = object.id;
                    detection.label = object.raw_label;
                    detection.x = object.position[0];
                    detection.y = object.position[1];
                    detection.z = object.position[2];
                    detection.confidence = object.confidence;
                    msg.objects.push_back(detection);
                }

                object_detection_pub_->publish(msg);
            }
        }

        // Annotate and publish image
        if (detection_annotation_.getNumSubscribers() > 0) {
            left_cv_ = slMat2cvMat(left_sl_);
            for (const auto& detection : detections) {
                cv::Rect r = get_rect(detection.box);
                cv::rectangle(left_cv_, r, cv::Scalar(0x27, 0xC1, 0x36), 2);
                cv::putText(left_cv_, std::to_string(static_cast<int>(detection.label)), cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
            }
            
            std_msgs::msg::Header header;
            cv_bridge::CvImage annotated_img(header, sensor_msgs::image_encodings::TYPE_8UC4, left_cv_);
            detection_annotation_.publish(annotated_img.toImageMsg());
        }
    }

    // Member variables
    sl::Camera zed_;
    Yolo detector_;
    sl::Mat left_sl_;
    cv::Mat left_cv_;
    sl::Objects objects_;
    sl::ObjectDetectionRuntimeParameters object_tracker_params_rt_;
    sl::Resolution display_resolution_;
    
    rclcpp::Publisher<zed_interfaces::msg::ObjectDetections>::SharedPtr object_detection_pub_;
    image_transport::Publisher detection_annotation_;
    rclcpp::TimerBase::SharedPtr timer_;
};




int main(int argc, char** argv) {
    /* ROS initialization */
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectDetectionNode>());

    // ros::Publisher object_detection_pub = n.advertise<zed_msgs::ObjectDetections>("/object_detection", 10);
    // ros::Rate loop_rate(66);

    // image_transport::ImageTransport it(n);
    // image_transport::Publisher detection_annotation = it.advertise("/object_detection/annotated", 1);

    // std::cout << "Initialized publishers" << std::endl;

    // sensor_msgs::CameraInfoPtr left_camera_info_msg;
    // left_camera_info_msg.reset(new sensor_msgs::CameraInfo());
    // std::string left_camera_frame_id = "zed2i_left_camera_optical_frame";

    /* Resolution calcualations */
    sl::Resolution resolution;

    /* ZED camera initializaion */
    // Opening the ZED camera before the model deserialization to avoid cuda context issue
    sl::Camera zed;
    sl::InitParameters init_parameters;
    init_parameters.sdk_verbose = true;
    init_parameters.input.setFromSerialNumber(20382332);
    init_parameters.depth_mode = sl::DEPTH_MODE::ULTRA;

    std::cout << std::to_string(zed.getCameraInformation().serial_number) << std::endl;

    // Open the camera
    auto returned_state = zed.open(init_parameters);
    if (returned_state != sl::ERROR_CODE::SUCCESS) {
        print("Camera Open", returned_state, "Exit program.");
        return EXIT_FAILURE;
    }
    zed.enablePositionalTracking();

    /* ZED object detection initialization */
    sl::ObjectDetectionParameters detection_parameters;
    detection_parameters.enable_tracking = true;
    //detection_parameters.enable_segmentation = false; // designed to give person pixel mask, ZED SDK 4 only
    detection_parameters.detection_model = sl::OBJECT_DETECTION_MODEL::CUSTOM_BOX_OBJECTS;
    returned_state = zed.enableObjectDetection(detection_parameters);
    if (returned_state != sl::ERROR_CODE::SUCCESS) {
        print("enableObjectDetection", returned_state, "\nExit program.");
        zed.close();
        return EXIT_FAILURE;
    }
    auto camera_config = zed.getCameraInformation().camera_configuration;
    sl::Resolution pc_resolution(std::min((int) camera_config.resolution.width, 720), std::min((int) camera_config.resolution.height, 404));
    auto camera_info = zed.getCameraInformation(pc_resolution).camera_configuration;

    std::cout << "Initialized ZED camera" << std::endl;

    /* Custom YOLOv8 model initialization */
    std::string engine_name = "";
    Yolo detector;
    if (argc > 0)
        engine_name = argv[1];
    else {
        std::cout << "Error: missing engine name as argument" << std::endl;
        return EXIT_FAILURE;
    }
    if (detector.init(engine_name)) {
        std::cerr << "Detector init failed!" << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "Initialized object detection" << std::endl;

    /* Object detection data initialization */
    auto display_resolution = zed.getCameraInformation().camera_configuration.resolution;
    sl::Mat left_sl, point_cloud;
    cv::Mat left_cv;
    sl::ObjectDetectionRuntimeParameters objectTracker_parameters_rt;
    sl::Objects objects;
    sl::Pose cam_w_pose;
    cam_w_pose.pose_data.setIdentity();

    while (ros::ok()) {
        if (zed.grab() == sl::ERROR_CODE::SUCCESS) {
            /* Left image */
            zed.retrieveImage(left_sl, sl::VIEW::LEFT);

            /* Object detections */
            publish_detections(
                detector,
                left_sl,
                left_cv,
                objectTracker_parameters_rt,
                objects,
                display_resolution,
                zed,
                object_detection_pub,
                detection_annotation
            );

            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    return 0;
}
