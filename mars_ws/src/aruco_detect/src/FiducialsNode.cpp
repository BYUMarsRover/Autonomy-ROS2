// Implementation of the FiducialNode functions
#include "FiducialsNode.h"

// Constructor implementation
FiducialsNode::FiducialsNode() : nh(), pnh("~"), it(nh) {
    frameNum = 0;

    // Camera intrinsics
    cameraMatrix = cv::Mat::zeros(3, 3, CV_64F);

    // Distortion coefficients
    distortionCoeffs = cv::Mat::zeros(1, 5, CV_64F);

    haveCamInfo = false;
    enable_detections = true;

    int dicno;

    detectorParams = new aruco::DetectorParameters();

    // Parameter retrieval from ROS
    pnh.param<bool>("publish_images", publish_images, false);
    pnh.param<double>("fiducial_len", fiducial_len, 0.14);
    pnh.param<int>("dictionary", dicno, 7);
    pnh.param<bool>("do_pose_estimation", doPoseEstimation, true);

    std::string str;
    std::vector<std::string> strs;

    // Handle ignored fiducials
    pnh.param<std::string>("ignore_fiducials", str, "");
    handleIgnoreString(str);

    // Handle fiducial length overrides
    pnh.param<std::string>("fiducial_len_override", str, "");
    boost::split(strs, str, boost::is_any_of(","));
    for (const std::string& element : strs) {
        if (element.empty()) continue;
        std::vector<std::string> parts;
        boost::split(parts, element, boost::is_any_of(":"));
        if (parts.size() == 2) {
            double len = std::stod(parts[1]);
            std::vector<std::string> range;
            boost::split(range, parts[0], boost::is_any_of("-"));
            if (range.size() == 2) {
                int start = std::stoi(range[0]);
                int end = std::stoi(range[1]);
                ROS_INFO("Setting fiducial id range %d - %d length to %f",
                         start, end, len);
                for (int j = start; j <= end; j++) {
                    fiducialLens[j] = len;
                }
            } else if (range.size() == 1) {
                int fid = std::stoi(range[0]);
                ROS_INFO("Setting fiducial id %d length to %f", fid, len);
                fiducialLens[fid] = len;
            } else {
                ROS_ERROR("Malformed fiducial_len_override: %s", element.c_str());
            }
        } else {
            ROS_ERROR("Malformed fiducial_len_override: %s", element.c_str());
        }
    }

    // ROS publishers
    image_pub = it.advertise("/fiducial_images", 1);
    vertices_pub = new ros::Publisher(nh.advertise<fiducial_msgs::FiducialArray>("fiducial_vertices", 1));
    pose_pub = new ros::Publisher(nh.advertise<fiducial_msgs::FiducialTransformArray>("fiducial_transforms", 1));

    // Marker dictionary setup
    static const aruco::Dictionary COMPETITION_MARKERS = aruco::Dictionary(
        Mat(11, (7 * 7 + 7) / 8, CV_8UC4, (uchar *)COMPETITION_MARKERS_7X7),
        7,
        3);
    dictionary = makePtr<aruco::Dictionary>(COMPETITION_MARKERS);

    // ROS subscribers
    img_sub = it.subscribe("camera", 1, &FiducialsNode::imageCallback, this);
    caminfo_sub = nh.subscribe("camera_info", 1, &FiducialsNode::camInfoCallback, this);
    ignore_sub = nh.subscribe("ignore_fiducials", 1, &FiducialsNode::ignoreCallback, this);

    // Service for enabling/disabling detections
    service_enable_detections = nh.advertiseService("enable_detections", &FiducialsNode::enableDetectionsCallback, this);

    // Parameter server setup
    callbackType = boost::bind(&FiducialsNode::configCallback, this, _1, _2);
    configServer.setCallback(callbackType);

    // Detector parameters
    // Set other parameters as needed...

    ROS_INFO("Aruco detection ready");
}

// Callback method implementations
void FiducialsNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
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

void FiducialsNode::camInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
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

void FiducialsNode::ignoreCallback(const std_msgs::msg::String::SharedPtr msg) {
    ignoreIds.clear();
    pnh.setParam("ignore_fiducials", msg.data);
    handleIgnoreString(msg.data);
}

bool FiducialsNode::enableDetectionsCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                               const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                               const std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
    enable_detections = req->data;

    if (enable_detections) {
        res->message = "Aruco detections have been enabled.";
        ROS_INFO("Aruco detections enabled.");
    } else {
        res->message = "Aruco detections have been disabled.";
        ROS_INFO("Aruco detections disabled.");
    }
    
    res->success = true; // Assuming the operation is always successful.
    return res->success; // Return the success status.
}


rcl_interfaces::msg::SetParametersResult FiducialsNode::parameterCallback(const std::vector<Parameter>& params) {
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

//Helper functions
void FiducialsNode::configCallback(aruco_detect::DetectorParamsConfig & config, uint32_t level) {
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

void handleIgnoreString(const std::string& str) {
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

