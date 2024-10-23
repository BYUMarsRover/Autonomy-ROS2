#include <iostream>
#include <fstream>
#include <signal.h>

#include "std_msgs/msg/int32.h"
#include "std_msgs/msg/string.h"

#include <UBLOX/ublox_ros.h>

namespace ublox_ros
{

UBLOX_ROS::UBLOX_ROS() : Node("ublox_ros") {

    // Connect ROS topics
    advertiseTopics();

    // Connect ROS services
    advertiseServices();

    // declare parameters

    std::map<std::string, std::string> string_params;
    std::map<std::string, int> int_params;
    std::map<std::string, double> double_params;

    string_params["arrowbase"] = "";
    string_params["arrowtip"]= "";
    string_params["serial_port"] = "/dev/ttyACM0";
    string_params["log_filename"] = "";
    string_params["base_type"] = "moving";
    string_params["local_host"] = "localhost";
    string_params["rover_host"] = "";
    string_params["base_host"] = "";
    string_params["local_host1"] = "localhost";
    string_params["base_host1"] = "localhost";

    int_params["message_rate"] = 5; //rate at which GNSS measurements are taken in hz
    int_params["rover_quantity"] = 0;
    int_params["GPS"] = 1;
    int_params["GLONAS"] = 0;
    int_params["BEIDOU"] = 0;
    int_params["GALILEO"] = 1;
    int_params["dynamic_model"] = 0;
    int_params["Surveytime"] = 120;
    int_params["Surveyacc"] = 500000;
    int_params["base_port"] = 16145;
    int_params["base_port1"] = 16145;

    double_params["base_lat"] = 0.0;
    double_params["base_lon"] = 0.0;
    double_params["base_alt"] = 0.0;

    
    this->declare_parameters("", string_params);
    this->declare_parameters("", int_params);
    this->declare_parameters("", double_params);

    this->declare_parameter<bool>("debug", false);

    this->declare_parameter<uint16_t>("local_port1", 16140);
    this->declare_parameter<uint16_t>("local_port", 16140);
    this->declare_parameter<uint16_t>("rover_port", 16145);

    
    //Get the serial port


    this->get_parameter<std::string>("serial_port", serial_port_);
    this->get_parameter<std::string>("log_filename", log_filename_);
    this->get_parameter<int>("message_rate", message_rate_); //rate at which GNSS measurements are taken in hz
    this->get_parameter<int>("rover_quantity", rover_quantity_);
    
    // Get Constallation settings
    

    this->get_parameter<int>("GPS", gps_);
    this->get_parameter<int>("GLONAS", glonas_);
    this->get_parameter<int>("BEIDOU", beidou_);
    this->get_parameter<int>("GALILEO", galileo_);
    this->get_parameter<uint8_t>("dynamic_model", dynamic_model_);
    
    std::cerr << "message_rate = " << message_rate_ << "\n";
    std::cerr << "rover_quantity = " << rover_quantity_ << "\n";
    std::cerr << "gps = " << gps_ << "\n";
    std::cerr << "glonas = " << glonas_ << "\n";
    std::cerr << "beidou = " << beidou_ << "\n";
    std::cerr << "galileo = " << galileo_ << "\n";
    constellation_.gps_enable = gps_;
    constellation_.glonas_enable = glonas_;
    constellation_.beidou_enable = beidou_;
    constellation_.galileo_enable = galileo_;

    // create the parser
    ublox_ = new ublox::UBLOX(serial_port_, message_rate_);

    // set up RTK
    // Base (n local_host n local_port, n rover_host, n rover_port)
    
    bool debug_param;
    this->get_parameter<bool>("debug", debug_param);

    std::string base_host_param;
    std::string rover_host_param;

    this->get_parameter<std::string>("base_host", base_host_param);
    this->get_parameter<std::string>("rover_host", rover_host_param);

    // set up RTK
    // Base (n local_host n local_port, n rover_host, n rover_port)
    if(debug_param)
    {
        std::cerr<<"DEBUG MODE\n";
    }
    else if (base_host_param == "")
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "This is the base function call! base_host: " << base_host_param << "!");
        initBase();
    }
    // Rover(1 local_host 1 local_port 1 base_host 1 base_port)
    else if (rover_host_param == "")
    {
        initRover();
    }
    // Brover(1 base_host 1 base_port n local_host n local_port n rover_host n rover_port)
    else if (base_host_param != "" && rover_host_param != "") 
    {
        initBrover();
    }
    else
    {
        std::cerr<<"Could not deduce base, rover, or brover\n";
    }
    
    // TODO: implement this chunck of code into ROS2
    // // Check if there is a arrow
    // if (nh_private_.hasParam("arrowbase") && nh_private_.hasParam("arrowtip")) {

    //   // If there is an arrow , then we need to subscribe to the base
    //   std::string arrowbase = nh_private_.param<std::string>("arrowbase", "/brover");
    //   // and tip of the arrow for /RelPos
    //   std::string arrowtip = nh_private_.param<std::string>("arrowtip", "/rover");

    //   // Call the first subscriber
    //   sub1 = nh_.subscribe(arrowbase+"/RelPos", 10, &UBLOX_ROS::cb_rov1, this);

    //   // Call the second subscriber
    //   sub2 = nh_.subscribe(arrowtip+"/RelPos", 10, &UBLOX_ROS::cb_rov2, this);

    //   // Make the arrow flag true. This flag is used in the relposCB function in
    //   // in order to determine if the vector math function needs to be called or
    //   // not.
    //   arrow_flag = true;
    // }


    // connect callbacks
    createCallback(ublox::CLASS_NAV, ublox::NAV_RELPOSNED, &UBLOX_ROS::relposCB, this);
    createCallback(ublox::CLASS_NAV, ublox::NAV_POSECEF, &UBLOX_ROS::posECEFCB, this);
    createCallback(ublox::CLASS_NAV, ublox::NAV_VELECEF, &UBLOX_ROS::velECEFCB, this);
    createCallback(ublox::CLASS_NAV, ublox::NAV_SVIN, &UBLOX_ROS::svinCB, this);
    createCallback(ublox::CLASS_RXM, ublox::RXM_RAWX, &UBLOX_ROS::obsCB, this);
    createCallback(ublox::CLASS_NAV, ublox::NAV_PVT, &UBLOX_ROS::pvtCB, this);
    createCallback(ublox::CLASS_RXM, ublox::RXM_RTCM, &UBLOX_ROS::rtcmInputCB, this);
    createCallback(ublox::CLASS_RXM, ublox::RXM_MEASX, &UBLOX_ROS::rxmMeasxCB, this);
    if (!log_filename_.empty())
    {
        ublox_->initLogFile(log_filename_);
        //ublox_->readFile(log_filename_);
    }
}

UBLOX_ROS::~UBLOX_ROS()
{
    if (ublox_)
        delete ublox_;
}

bool UBLOX_ROS::evalF9PID(uint8_t f9pID)
{
    // RCLCPP_INFO_STREAM(this->get_logger(), "The f9pID is: " << int(f9pID));
    switch(f9pID)
    {
        case 0:
            pvt_ptr_ = &pvt_msg_;
            ecef_ptr_ = &ecef_msg_;
            ecef_pos_tow_ptr_ = &ecef_pos_tow_;
            ecef_vel_tow_ptr_ = &ecef_vel_tow_;
            pvt_tow_ptr_ = &pvt_tow_;
            return true;
            break;
        case 1:
            pvt_ptr_ = &base_pvt_msg_;
            ecef_ptr_ = &base_ecef_msg_;
            ecef_pos_tow_ptr_ = &base_ecef_pos_tow_;
            ecef_vel_tow_ptr_ = &base_ecef_vel_tow_;
            pvt_tow_ptr_ = &base_pvt_tow_;
            return false;   // TODO: make sure a) this is the correct one to change to false and b) make sure it didn't break anything else
            break;
        default:
            return false;
            break;
    }
}

}
