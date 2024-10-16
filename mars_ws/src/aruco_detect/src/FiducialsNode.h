#ifndef FIDUCIALS_NODE_H  // Include guard to prevent multiple inclusion
#define FIDUCIALS_NODE_H

#pragma once
#include <rclcpp/rclcpp.hpp> 
#include <std_msgs/String.h>
#include <rclcpp/rclcpp.hpp> 
#include <vector>
#include <string>
using namespace std;


class FiducialsNode {
  private:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vertices_pub;         //  publishers in ROS2 to send message - probably sends information about the fiducial markers' vertices
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pose_pub;             //  publishers in ROS2 to send message - likely publishes the pose of fiducial markers

    // Subscriber listens to messages being published on a particular topic.
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr caminfo_sub;                 // listens to camera info messages (probably camera calibration or camera parameters).
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ignore_sub;                  // listens to a string message that likely contains a list of marker IDs to ignore.
    image_transport::Subscriber img_sub;                                                // is an image transport subscriber, which listens to image messages, likely from a camera feed. It receives sensor_msgs::Image messages and converts them into OpenCV images.

    // Service server. In ROS, services are like functions that can be called remotely. This service likely toggles the detection of fiducial markers on or off.
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_enable_detections;

    // if set, we publish the images that contain fiducials
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

    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    image_transport::Publisher image_pub;

    cv::Ptr<aruco::DetectorParameters> detectorParams;
    cv::Ptr<aruco::Dictionary> dictionary;

    void handleIgnoreString(const std::string& str);

    void estimatePoseSingleMarkers(const vector<int> &ids,
                                   const vector<vector<Point2f > >&corners,
                                   float markerLength,
                                   const cv::Mat &cameraMatrix,
                                   const cv::Mat &distCoeffs,
                                   vector<Vec3d>& rvecs, vector<Vec3d>& tvecs,
                                   vector<double>& reprojectionError);


    void ignoreCallback(const std_msgs::String &msg);
    void imageCallback(const sensor_msgs::ImageConstPtr &msg);
    void camInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &msg);
    void configCallback(aruco_detect::DetectorParamsConfig &config, uint32_t level);

    bool enableDetectionsCallback(std_srvs::SetBool::Request &req,
                                std_srvs::SetBool::Response &res);

    // COMMENTED OUT CODE BELOW AS dynamic_reconfigure is not available for ROS2
    // dynamic_reconfigure::Server<aruco_detect::DetectorParamsConfig> configServer; 
    // dynamic_reconfigure::Server<aruco_detect::DetectorParamsConfig>::CallbackType callbackType; 

    // NOTE: Parameter callback for dynamic changes. This function is called whenever a parameter is updated at runtime. 
    // Inside this callback, you check which parameters were updated and modify the node’s internal state accordingly.
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



    void configCallback(aruco_detect::DetectorParamsConfig & config, uint32_t level)
{
    /* Don't load initial config, since it will overwrite the rosparam settings */
    if (level == 0xFFFFFFFF) {
        return;
    }

    detectorParams->adaptiveThreshConstant = config.adaptiveThreshConstant;
    detectorParams->adaptiveThreshWinSizeMin = config.adaptiveThreshWinSizeMin;
    detectorParams->adaptiveThreshWinSizeMax = config.adaptiveThreshWinSizeMax;
    detectorParams->adaptiveThreshWinSizeStep = config.adaptiveThreshWinSizeStep;
    detectorParams->cornerRefinementMaxIterations = config.cornerRefinementMaxIterations;
    detectorParams->cornerRefinementMinAccuracy = config.cornerRefinementMinAccuracy;
    detectorParams->cornerRefinementWinSize = config.cornerRefinementWinSize;
#if CV_MINOR_VERSION==2
    detectorParams->doCornerRefinement = config.doCornerRefinement;
#else
    if (config.doCornerRefinement) {
       if (config.cornerRefinementSubpix) {
         detectorParams->cornerRefinementMethod = aruco::CORNER_REFINE_SUBPIX;
       }
       else {
         detectorParams->cornerRefinementMethod = aruco::CORNER_REFINE_CONTOUR;
       }
    }
    else {
       detectorParams->cornerRefinementMethod = aruco::CORNER_REFINE_NONE;
    }
#endif
    detectorParams->errorCorrectionRate = config.errorCorrectionRate;
    detectorParams->minCornerDistanceRate = config.minCornerDistanceRate;
    detectorParams->markerBorderBits = config.markerBorderBits;
    detectorParams->maxErroneousBitsInBorderRate = config.maxErroneousBitsInBorderRate;
    detectorParams->minDistanceToBorder = config.minDistanceToBorder;
    detectorParams->minMarkerDistanceRate = config.minMarkerDistanceRate;
    detectorParams->minMarkerPerimeterRate = config.minMarkerPerimeterRate;
    detectorParams->maxMarkerPerimeterRate = config.maxMarkerPerimeterRate;
    detectorParams->minOtsuStdDev = config.minOtsuStdDev;
    detectorParams->perspectiveRemoveIgnoredMarginPerCell = config.perspectiveRemoveIgnoredMarginPerCell;
    detectorParams->perspectiveRemovePixelPerCell = config.perspectiveRemovePixelPerCell;
    detectorParams->polygonalApproxAccuracyRate = config.polygonalApproxAccuracyRate;
}

void ignoreCallback(const std_msgs::String& msg)
{
    ignoreIds.clear();
    pnh.setParam("ignore_fiducials", msg.data);
    handleIgnoreString(msg.data);
}

void camInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
    if (haveCamInfo) {
        return;
    }

    if (msg->K != boost::array<double, 9>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0})) {
        for (int i=0; i<3; i++) {
            for (int j=0; j<3; j++) {
                cameraMatrix.at<double>(i, j) = msg->K[i*3+j];
            }
        }

        for (int i=0; i<5; i++) {
            distortionCoeffs.at<double>(0,i) = msg->D[i];
        }

        haveCamInfo = true;
        frameId = msg->header.frame_id;
    }
    else {
        ROS_WARN("%s", "CameraInfo message has invalid intrinsics, K matrix all zeros");
    }
}

void imageCallback(const sensor_msgs::ImageConstPtr & msg) {
    if (enable_detections == false) {
        return; //return without doing anything
    }

    //ROS_INFO("Got image %d", msg->header.seq);
    frameNum++;

    cv_bridge::CvImagePtr cv_ptr;

    fiducial_msgs::FiducialTransformArray fta;
    fta.header.stamp = msg->header.stamp;
    fta.header.frame_id = frameId;
    fta.image_seq = msg->header.seq;

    fiducial_msgs::FiducialArray fva;
    fva.header.stamp = msg->header.stamp;
    fva.header.frame_id =frameId;
    fva.image_seq = msg->header.seq;

    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        vector <int>  ids;
        vector <vector <Point2f> > corners, rejected;
        vector <Vec3d>  rvecs, tvecs;

        aruco::detectMarkers(cv_ptr->image, dictionary, corners, ids, detectorParams);
        if ((int)ids.size() > 0) {
            ROS_INFO("Detected %d markers", (int)ids.size());
        }

        for (size_t i=0; i<ids.size(); i++) {
	    if (std::count(ignoreIds.begin(), ignoreIds.end(), ids[i]) != 0) {
	        ROS_INFO("Ignoring id %d", ids[i]);
	        continue;
	    }
            fiducial_msgs::Fiducial fid;
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

        if(ids.size() > 0) {
            aruco::drawDetectedMarkers(cv_ptr->image, corners, ids);
        }

        if (doPoseEstimation) {
            if (!haveCamInfo) {
                if (frameNum > 5) {
                    ROS_ERROR("No camera intrinsics");
                }
                return;
            }

            vector <double>reprojectionError;
            estimatePoseSingleMarkers(ids, corners, (float)fiducial_len,
                                      cameraMatrix, distortionCoeffs,
                                      rvecs, tvecs,
                                      reprojectionError);

            for (size_t i=0; i<ids.size(); i++) {
                aruco::drawAxis(cv_ptr->image, cameraMatrix, distortionCoeffs,
                                rvecs[i], tvecs[i], (float)fiducial_len);

                ROS_INFO("Detected id %d T %.2f %.2f %.2f R %.2f %.2f %.2f", ids[i],
                         tvecs[i][0], tvecs[i][1], tvecs[i][2],
                         rvecs[i][0], rvecs[i][1], rvecs[i][2]);

                if (std::count(ignoreIds.begin(), ignoreIds.end(), ids[i]) != 0) {
                    ROS_INFO("Ignoring id %d", ids[i]);
                    continue;
                }

                double angle = norm(rvecs[i]);
                Vec3d axis = rvecs[i] / angle;
                ROS_INFO("angle %f axis %f %f %f",
                         angle, axis[0], axis[1], axis[2]);

                fiducial_msgs::FiducialTransform ft;
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
    catch(cv_bridge::Exception & e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    catch(cv::Exception & e) {
        ROS_ERROR("cv exception: %s", e.what());
    }
}

void estimatePoseSingleMarkers(const vector<int> &ids,
                                const vector<vector<Point2f > >&corners,
                                float markerLength,
                                const cv::Mat &cameraMatrix,
                                const cv::Mat &distCoeffs,
                                vector<Vec3d>& rvecs, vector<Vec3d>& tvecs,
                                vector<double>& reprojectionError) {

    CV_Assert(markerLength > 0);

    vector<Point3f> markerObjPoints;
    int nMarkers = (int)corners.size();
    rvecs.reserve(nMarkers);
    tvecs.reserve(nMarkers);
    reprojectionError.reserve(nMarkers);

    // for each marker, calculate its pose
    for (int i = 0; i < nMarkers; i++) {
       double fiducialSize = markerLength;

       std::map<int, double>::iterator it = fiducialLens.find(ids[i]);
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



void handleIgnoreString(const std::string& str)
{
    /*
    ignogre fiducials can take comma separated list of individual
    fiducial ids or ranges, eg "1,4,8,9-12,30-40"
    */
    std::vector<std::string> strs;
    boost::split(strs, str, boost::is_any_of(","));
    for (const string& element : strs) {
        if (element == "") {
           continue;
        }
        std::vector<std::string> range;
        boost::split(range, element, boost::is_any_of("-"));
        if (range.size() == 2) {
           int start = std::stoi(range[0]);
           int end = std::stoi(range[1]);
           ROS_INFO("Ignoring fiducial id range %d to %d", start, end);
           for (int j=start; j<=end; j++) {
               ignoreIds.push_back(j);
           }
        }
        else if (range.size() == 1) {
           int fid = std::stoi(range[0]);
           ROS_INFO("Ignoring fiducial id %d", fid);
           ignoreIds.push_back(fid);
        }
        else {
           ROS_ERROR("Malformed ignore_fiducials: %s", element.c_str());
        }
    }
}

bool enableDetectionsCallback(std_srvs::SetBool::Request &req,
                                std_srvs::SetBool::Response &res)
{
    enable_detections = req.data;
    if (enable_detections){
        res.message = "Enabled aruco detections.";
        ROS_INFO("Enabled aruco detections.");
    }
    else {
        res.message = "Disabled aruco detections.";
        ROS_INFO("Disabled aruco detections.");
    }
    
    res.success = true;
    return true;
}


FiducialsNode() : nh(), pnh("~"), it(nh)
{
    frameNum = 0;

    // Camera intrinsics
    cameraMatrix = cv::Mat::zeros(3, 3, CV_64F);

    // distortion coefficients
    distortionCoeffs = cv::Mat::zeros(1, 5, CV_64F);

    haveCamInfo = false;
    enable_detections = true;

    int dicno;

    detectorParams = new aruco::DetectorParameters();

    pnh.param<bool>("publish_images", publish_images, false);
    pnh.param<double>("fiducial_len", fiducial_len, 0.14);
    pnh.param<int>("dictionary", dicno, 7);
    pnh.param<bool>("do_pose_estimation", doPoseEstimation, true);

    std::string str;
    std::vector<std::string> strs;

    pnh.param<string>("ignore_fiducials", str, "");
    handleIgnoreString(str);

    /*
    fiducial size can take comma separated list of size: id or size: range,
    e.g. "200.0: 12, 300.0: 200-300"
    */
    pnh.param<string>("fiducial_len_override", str, "");
    boost::split(strs, str, boost::is_any_of(","));
    for (const string& element : strs) {
        if (element == "") {
           continue;
        }
        std::vector<std::string> parts;
        boost::split(parts, element, boost::is_any_of(":"));
        if (parts.size() == 2) {
            double len = std::stod(parts[1]);
            std::vector<std::string> range;
            boost::split(range, element, boost::is_any_of("-"));
            if (range.size() == 2) {
               int start = std::stoi(range[0]);
               int end = std::stoi(range[1]);
               ROS_INFO("Setting fiducial id range %d - %d length to %f",
                        start, end, len);
               for (int j=start; j<=end; j++) {
                   fiducialLens[j] = len;
               }
            }
            else if (range.size() == 1){
               int fid = std::stoi(range[0]);
               ROS_INFO("Setting fiducial id %d length to %f", fid, len);
               fiducialLens[fid] = len;
            }
            else {
               ROS_ERROR("Malformed fiducial_len_override: %s", element.c_str());
            }
        }
        else {
           ROS_ERROR("Malformed fiducial_len_override: %s", element.c_str());
        }
    }

    image_pub = it.advertise("/fiducial_images", 1);

    vertices_pub = new ros::Publisher(nh.advertise<fiducial_msgs::FiducialArray>("fiducial_vertices", 1));

    pose_pub = new ros::Publisher(nh.advertise<fiducial_msgs::FiducialTransformArray>("fiducial_transforms", 1));

    //dictionary = aruco::getPredefinedDictionary(7);

    static const aruco::Dictionary COMPETITION_MARKERS = aruco::Dictionary(
        Mat(
            11,
            //(5 * 5 + 7) / 8,
	    (7 * 7 + 7) / 8,		// 7 bits x 7 bits + 7 to avoid integer truncation / 8 bits.
            CV_8UC4,
            //(uchar *)COMPETITION_MARKERS_5X5_20_BYTES),
	    (uchar *)COMPETITION_MARKERS_7X7),
        //5,
	7,
        3);

    dictionary = makePtr<aruco::Dictionary>(COMPETITION_MARKERS);
    

    img_sub = it.subscribe("camera", 1,
                        &imageCallback, this);

    caminfo_sub = nh.subscribe("camera_info", 1,
                    &camInfoCallback, this);

    ignore_sub = nh.subscribe("ignore_fiducials", 1,
                              &ignoreCallback, this);

    service_enable_detections = nh.advertiseService("enable_detections",
                        &enableDetectionsCallback, this);

    callbackType = boost::bind(&configCallback, this, _1, _2);
    configServer.setCallback(callbackType);

    pnh.param<double>("adaptiveThreshConstant", detectorParams->adaptiveThreshConstant, 7);
    pnh.param<int>("adaptiveThreshWinSizeMax", detectorParams->adaptiveThreshWinSizeMax, 53); /* defailt 23 */
    pnh.param<int>("adaptiveThreshWinSizeMin", detectorParams->adaptiveThreshWinSizeMin, 3);
    pnh.param<int>("adaptiveThreshWinSizeStep", detectorParams->adaptiveThreshWinSizeStep, 4); /* default 10 */
    pnh.param<int>("cornerRefinementMaxIterations", detectorParams->cornerRefinementMaxIterations, 30);
    pnh.param<double>("cornerRefinementMinAccuracy", detectorParams->cornerRefinementMinAccuracy, 0.01); /* default 0.1 */
    pnh.param<int>("cornerRefinementWinSize", detectorParams->cornerRefinementWinSize, 5);
#if CV_MINOR_VERSION==2
    pnh.param<bool>("doCornerRefinement",detectorParams->doCornerRefinement, true); /* default false */
#else
    bool doCornerRefinement = true;
    pnh.param<bool>("doCornerRefinement", doCornerRefinement, true);
    if (doCornerRefinement) {
       bool cornerRefinementSubPix = true;
       pnh.param<bool>("cornerRefinementSubPix", cornerRefinementSubPix, true);
       if (cornerRefinementSubPix) {
         detectorParams->cornerRefinementMethod = aruco::CORNER_REFINE_SUBPIX;
       }
       else {
         detectorParams->cornerRefinementMethod = aruco::CORNER_REFINE_CONTOUR;
       }
    }
    else {
       detectorParams->cornerRefinementMethod = aruco::CORNER_REFINE_NONE;
    }
#endif
    pnh.param<double>("errorCorrectionRate", detectorParams->errorCorrectionRate , 0.6);
    pnh.param<double>("minCornerDistanceRate", detectorParams->minCornerDistanceRate , 0.05);
    pnh.param<int>("markerBorderBits", detectorParams->markerBorderBits, 1);
    pnh.param<double>("maxErroneousBitsInBorderRate", detectorParams->maxErroneousBitsInBorderRate, 0.04);
    pnh.param<int>("minDistanceToBorder", detectorParams->minDistanceToBorder, 3);
    pnh.param<double>("minMarkerDistanceRate", detectorParams->minMarkerDistanceRate, 0.05);
    pnh.param<double>("minMarkerPerimeterRate", detectorParams->minMarkerPerimeterRate, 0.1); /* default 0.3 */
    pnh.param<double>("maxMarkerPerimeterRate", detectorParams->maxMarkerPerimeterRate, 4.0);
    pnh.param<double>("minOtsuStdDev", detectorParams->minOtsuStdDev, 5.0);
    pnh.param<double>("perspectiveRemoveIgnoredMarginPerCell", detectorParams->perspectiveRemoveIgnoredMarginPerCell, 0.13);
    pnh.param<int>("perspectiveRemovePixelPerCell", detectorParams->perspectiveRemovePixelPerCell, 8);
    pnh.param<double>("polygonalApproxAccuracyRate", detectorParams->polygonalApproxAccuracyRate, 0.01); /* default 0.05 */

    ROS_INFO("Aruco detection ready");
}

};


#endif  // FIDUCIALS_NODE_H