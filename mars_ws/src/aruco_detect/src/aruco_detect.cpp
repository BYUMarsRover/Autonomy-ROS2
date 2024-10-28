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

#include <assert.h>
#include <sys/time.h>
#include <unistd.h>
#include "FiducialsNode.h"

#include <rclcpp/rclcpp.hpp> // #include <ros/ros.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <visualization_msgs/Marker.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <dynamic_reconfigure/server.h> // This is not available in ROS2

#include <std_srvs/srv/set_bool.hpp> // #include <std_srvs/SetBool.h>
#include <std_msgs/String.h>

#include "rover_msgs/msg/FidudcialData.hpp"

#include "aruco_detect/DetectorParamsConfig.h"

#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>

#include <list>
#include <string>
#include <boost/algorithm/string.hpp>

using namespace std;
using namespace cv;

//This set is for the competition tags (two black borders)
static unsigned char COMPETITION_MARKERS_7X7[][7][4] = 
    {{
	//leg1
	{  0, 217, 178, 167},
	{207, 128, 0,   0  },
	{249, 176, 230, 207},
	{128, 0,   0,   249},
	{242, 166, 205, 128},
	{0,   0,   249, 179},
	{134, 207, 128, 0  },
    },
    {
	//leg2
	{0,   217, 178, 161},
	{142, 128, 0,   0  },
	{233, 160, 230, 78 },
	{128, 0,   0,   184},
	{194, 166, 205, 128},
	{0,   0,   185, 51 },
	{130, 203, 128, 0  },
    },
    {
	//leg3
	{0,   217, 178, 165},
	{139, 0,   0,   0 },
	{225, 176, 230, 15 },
	{128, 0,   0,   104},
	{210, 166, 205, 128},
	{0,   0,    248, 51},
	{134, 195,  128, 0 },
    },
    {
	//leg4l
	{0,  217, 178, 163},
	{202, 0, 0, 0},
	{241, 160, 230, 142},
	{128,0, 0, 41},
	{226, 166, 205, 128},
	{0, 0, 184, 179},
	{130, 199, 128, 0},
    },
    {
	//leg4r
	{0,   217, 178, 163},
	{135, 0,   0,   0 },
	{225, 176, 230, 206 },
	{0, 0,   0,   112},
	{226, 166, 205, 128},
	{0,   0,    57, 179},
	{134, 195,  128, 0 },
    },
    {
	//leg5l
	{0,   217, 178, 165},
	{198, 0,   0,   0 },
	{241, 160, 230, 79 },
	{0, 0,   0,   49},
	{210, 166, 205, 128},
	{0,   0,    121, 51},
	{130, 199,  128, 0 },
    },
    {
	//leg5r
	{0,   217, 178, 161},
	{195, 128,   0,   0 },
	{249, 176, 230, 14 },
	{0, 0,   0,   225},
	{194, 166, 205, 128},
	{0,   0,    56, 51},
	{134, 207,  128, 0 }, 
    },
    {
	//leg6l
	{0,   217, 178, 167},
	{130, 128,   0,   0 },
	{233, 160, 230, 143 },
	{0, 0,   0,   160},
	{242, 166, 205, 128},
	{0,   0,    120, 179},
	{130, 203,  128, 0 },
    },
    {
	//leg6r
	{0,   217, 178, 161},
	{79, 0,   0,   0 },
	{241, 144, 230, 78 },
	{128, 0,   0,   121},
	{66, 166, 205, 128},
	{0,   0,    185, 51},
	{132, 199,  128, 0 },
    },
    {
	//leg7l
	{0,   217, 178, 167},
	{14, 0,   0,   0 },
	{225, 128, 230, 207 },
	{128, 0,   0,   56},
	{214, 166, 205, 128},
	{0,   0,    249, 179},
	{128, 195,  128, 0 },
    },
    {
	//leg7r
	{0,   217, 178, 163},
	{11, 128,   0,   0 },
	{233, 144, 230, 142 },
	{128, 0,   0,   232},
	{98, 166, 205, 128},
	{0,   0,    184, 179},
	{132, 203,  128, 0 },
    }};

//This set is for tags with one black border (not competition tags)
static unsigned char COMPETITION_MARKERS_5X5_20_BYTES[][4][4] =
    {{
         //leg1
         {222, 235, 255, 1},
         {254, 207, 191, 1},
         {255, 235, 189, 1},
         {254, 249, 191, 1},
     },
     {
         //leg2
         {222, 234, 110, 1},
         {238, 143, 158, 1},
         {187,  43, 189, 1},
         {188, 248, 187, 1},
     },
     {
         //leg3
         {222, 235, 107, 0},
         {230, 207, 143, 1},
         {107, 107, 189, 1},
         {248, 249, 179, 1},
     },
     {
         //leg4l
         {222, 234, 250, 0},
         {246, 143, 174, 1},
         {47, 171, 189, 1},
         {186, 248, 183, 1},
     },
     {
         //leg4r
         {222, 234, 231, 0},
         {230, 207, 190, 0},
         {115, 171, 189, 1},
         {62, 249, 179, 1},
     },
     {
         //leg5l
         {222, 235, 118, 0},
         {246, 143, 159, 0},
         {55, 107, 189, 1},
         {124, 248, 183, 1},
     },
     {
         //leg5r
         {222, 234, 115, 1},
         {254, 207, 142, 0},
         {231, 43, 189, 1},
         {56, 249, 191, 1},
     },
     {
         //leg6l
         {222, 235, 226, 1},
         {238, 143, 175, 0},
         {163, 235, 189, 1},
         {122, 248, 187, 1},
     },
     {
         //leg6r
         {222, 234, 95, 0},
         {246, 79, 158, 1},
         {125, 43, 189, 1},
         {188, 249, 55, 1},
     },
     {
         //leg7l
         {222, 235, 206, 0},
         {230, 15, 191, 1},
         {57, 235, 189, 1},
         {254, 248, 51, 1},
     },
     {
         //leg7r
         {222, 234, 203, 1},
         {238, 79, 174, 1},
         {233, 171, 189, 1},
         {186, 249, 59, 1},
     }};


/**
  * @brief Return object points for the system centered in a single marker, given the marker length
  */
static void getSingleMarkerObjectPoints(float markerLength, vector<Point3f>& objPoints) {

    CV_Assert(markerLength > 0);

    // set coordinate system in the middle of the marker, with Z pointing out
    objPoints.clear();
    objPoints.push_back(Vec3f(-markerLength / 2.f, markerLength / 2.f, 0));
    objPoints.push_back(Vec3f( markerLength / 2.f, markerLength / 2.f, 0));
    objPoints.push_back(Vec3f( markerLength / 2.f,-markerLength / 2.f, 0));
    objPoints.push_back(Vec3f(-markerLength / 2.f,-markerLength / 2.f, 0));
}

// Euclidean distance between two points
static double dist(const cv::Point2f &p1, const cv::Point2f &p2)
{
    double x1 = p1.x;
    double y1 = p1.y;
    double x2 = p2.x;
    double y2 = p2.y;

    double dx = x1 - x2;
    double dy = y1 - y2;

    return sqrt(dx*dx + dy*dy);
}

// Compute area in image of a fiducial, using Heron's formula
// to find the area of two triangles
static double calcFiducialArea(const std::vector<cv::Point2f> &pts)
{
    const Point2f &p0 = pts.at(0);
    const Point2f &p1 = pts.at(1);
    const Point2f &p2 = pts.at(2);
    const Point2f &p3 = pts.at(3);

    double a1 = dist(p0, p1);
    double b1 = dist(p0, p3);
    double c1 = dist(p1, p3);

    double a2 = dist(p1, p2);
    double b2 = dist(p2, p3);
    double c2 = c1;

    double s1 = (a1 + b1 + c1) / 2.0;
    double s2 = (a2 + b2 + c2) / 2.0;

    a1 = sqrt(s1*(s1-a1)*(s1-b1)*(s1-c1));
    a2 = sqrt(s2*(s2-a2)*(s2-b2)*(s2-c2));
    return a1+a2;
}

// estimate reprojection error
static double getReprojectionError(const vector<Point3f> &objectPoints,
                            const vector<Point2f> &imagePoints,
                            const Mat &cameraMatrix, const Mat  &distCoeffs,
                            const Vec3d &rvec, const Vec3d &tvec) {

    vector<Point2f> projectedPoints;

    cv::projectPoints(objectPoints, rvec, tvec, cameraMatrix,
                      distCoeffs, projectedPoints);

    // calculate RMS image error
    double totalError = 0.0;
    for (unsigned int i=0; i<objectPoints.size(); i++) {
        double error = dist(imagePoints[i], projectedPoints[i]);
        totalError += error*error;
    }
    double rerror = totalError/(double)objectPoints.size();
    return rerror;
}


int main(int argc, char ** argv) {
    ros::init(argc, argv, "aruco_detect");

    FiducialsNode* node = new FiducialsNode();

    ros::spin();

    return 0;
}
