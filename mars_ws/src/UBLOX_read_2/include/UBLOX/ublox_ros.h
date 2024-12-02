#ifndef UBLOX_ROS_H
#define UBLOX_ROS_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>

#include "UBLOX/ublox.h"

#include "ublox_read_2/msg/pos_vel_ecef.hpp"
#include "ublox_read_2/msg/position_velocity_time.hpp"
#include "ublox_read_2/msg/rel_pos.hpp"
#include "ublox_read_2/msg/rel_pos_flags.hpp"
#include "ublox_read_2/msg/rtcm_input.hpp"
#include "ublox_read_2/msg/survey_status.hpp"
#include "ublox_read_2/msg/ephemeris.hpp"
#include "ublox_read_2/msg/glonass_ephemeris.hpp"
#include "ublox_read_2/msg/observation.hpp"
#include "ublox_read_2/msg/obs_vec.hpp"
#include "ublox_read_2/msg/cfg_val_get_type.hpp"
#include "ublox_read_2/msg/satellite.hpp"
#include "ublox_read_2/msg/satellite_status.hpp"

#include "ublox_read_2/srv/cfg_val_get.hpp"
#include "ublox_read_2/srv/cfg_val_get_all.hpp"
#include "ublox_read_2/srv/cfg_val_del.hpp"
#include "ublox_read_2/srv/cfg_val_set.hpp"
#include "ublox_read_2/srv/cfg_reset.hpp"
#include "ublox_read_2/srv/get_version.hpp"
#include "ublox_read_2/srv/init_module.hpp"

namespace ublox_ros
{

class UBLOX_ROS : public rclcpp::Node
{
public:
    UBLOX_ROS();
    ~UBLOX_ROS();

private:
    ublox::UBLOX* ublox_ = nullptr;

    rclcpp::Publisher<ublox_read_2::msg::PositionVelocityTime>::SharedPtr pvt_pub_;
    rclcpp::Publisher<ublox_read_2::msg::SurveyStatus>::SharedPtr survey_status_pub_;
    rclcpp::Publisher<ublox_read_2::msg::RelPos>::SharedPtr relpos_pub_;
    rclcpp::Publisher<ublox_read_2::msg::RelPosFlags>::SharedPtr relposflag_pub_;
    rclcpp::Publisher<ublox_read_2::msg::PosVelEcef>::SharedPtr ecef_pub_;
    // rclcpp::Publisher<ublox_read_2::msg::NavSatFix>::SharedPtr nav_sat_fix_pub_;
    // rclcpp::Publisher<sensor_msgs::msg::NavSatStatus>::SharedPtr nav_sat_status_pub_;
    rclcpp::Publisher<ublox_read_2::msg::Ephemeris>::SharedPtr eph_pub_;
    rclcpp::Publisher<ublox_read_2::msg::GlonassEphemeris>::SharedPtr geph_pub_;
    rclcpp::Publisher<ublox_read_2::msg::ObsVec>::SharedPtr obs_pub_;
    rclcpp::Publisher<ublox_read_2::msg::RTCMInput>::SharedPtr rtcm_input_pub_;
    rclcpp::Publisher<ublox_read_2::msg::SatelliteStatus>::SharedPtr sat_status_pub_;

    rclcpp::Publisher<ublox_read_2::msg::PosVelEcef>::SharedPtr base_ecef_pub_;
    rclcpp::Publisher<ublox_read_2::msg::PositionVelocityTime>::SharedPtr base_pvt_pub_;
    rclcpp::Publisher<ublox_read_2::msg::PosVelEcef>::SharedPtr curr_ecef_pub_;
    rclcpp::Publisher<ublox_read_2::msg::PositionVelocityTime>::SharedPtr curr_pvt_pub_;

    /**
     * @brief Callback for filling a PosVelTime ROS message from a UBX callback
     */
    void pvtCB(const ublox::UBX_message_t &ubxmsg, uint8_t f9pID=0);
    /**
     * @brief Callback for filling a RelPos ROS message from a UBX callback
     */
    void relposCB(const ublox::UBX_message_t &ubx_msg, uint8_t f9pID=0);
    /**
     * @brief Callback for filling a PosVelEcef ROS message with position data from a UBX callback
     */
    void posECEFCB(const ublox::UBX_message_t &ubx_msg, uint8_t f9pID=0);
    /**
     * @brief Callback for filling a PosVelEcef ROS message with velocity data from a UBX callback
     */
    void velECEFCB(const ublox::UBX_message_t &ubx_msg, uint8_t f9pID=0);
    /**
     * @brief Callback for filling a SurveyStatus ROS message from a UBX callback
     */
    void svinCB(const ublox::UBX_message_t &ubx_msg, uint8_t f9pID=0);
    /**
     * @brief Callback for filling a Observation ROS message from a UBX callback
     */
    void obsCB(const ublox::UBX_message_t &ubx_msg, uint8_t f9pID=0);
    /**
     * @brief Callback for filling a RXM-RTCM ROS message from a UBX callback
     */
    void rtcmInputCB(const ublox::UBX_message_t &ubx_msg, uint8_t f9pID=0);
    /**
     * @brief Callback for filling a RXM-MEASX ROS message from a UBX callback
     */
    void rxmMeasxCB(const ublox::UBX_message_t &ubx_msg, uint8_t f9pID=0);

    void ephCB(const Ephemeris& eph);
    void gephCB(const GlonassEphemeris& eph);

    void cfgValGet(const std::shared_ptr<ublox_read_2::srv::CfgValGet::Request> req, std::shared_ptr<ublox_read_2::srv::CfgValGet::Response> res);
    void cfgValGetAll(std::shared_ptr<ublox_read_2::srv::CfgValGetAll::Request> req, std::shared_ptr<ublox_read_2::srv::CfgValGetAll::Response> res);
    void cfgValDel(std::shared_ptr<ublox_read_2::srv::CfgValDel::Request> req, std::shared_ptr<ublox_read_2::srv::CfgValDel::Response> res);
    void cfgValSet(std::shared_ptr<ublox_read_2::srv::CfgValSet::Request> req, std::shared_ptr<ublox_read_2::srv::CfgValSet::Response> res);
    void cfgReset(std::shared_ptr<ublox_read_2::srv::CfgReset::Request> req, std::shared_ptr<ublox_read_2::srv::CfgReset::Response> res);
    void initModule(std::shared_ptr<ublox_read_2::srv::InitModule::Request> req, std::shared_ptr<ublox_read_2::srv::InitModule::Response> res);
    void getVersion(std::shared_ptr<ublox_read_2::srv::GetVersion::Request> req, std::shared_ptr<ublox_read_2::srv::GetVersion::Response> res);
    rclcpp::Service<ublox_read_2::srv::CfgValGet>::SharedPtr cfg_val_get_;
    rclcpp::Service<ublox_read_2::srv::CfgValGetAll>::SharedPtr cfg_val_get_all_;
    rclcpp::Service<ublox_read_2::srv::CfgValDel>::SharedPtr cfg_val_del_;
    rclcpp::Service<ublox_read_2::srv::CfgValSet>::SharedPtr cfg_val_set_;
    rclcpp::Service<ublox_read_2::srv::CfgReset>::SharedPtr cfg_reset_;
    rclcpp::Service<ublox_read_2::srv::InitModule>::SharedPtr init_module_;
    rclcpp::Service<ublox_read_2::srv::GetVersion>::SharedPtr get_version_;

    uint32_t ecef_pos_tow_;
    uint32_t ecef_vel_tow_;
    uint32_t pvt_tow_;

    uint32_t base_ecef_pos_tow_;
    uint32_t base_ecef_vel_tow_;
    uint32_t base_pvt_tow_;

    uint32_t *ecef_pos_tow_ptr_;
    uint32_t *ecef_vel_tow_ptr_;
    uint32_t *pvt_tow_ptr_;

    uint32_t pvt_week_;

    std::string serial_port_;
    std::string log_filename_;
    int message_rate_;
    int rover_quantity_;

    int gps_;
    int glonas_;
    int beidou_;
    int galileo_;
    ublox::GNSS_CONSTELLATION_t constellation_;
    uint8_t dynamic_model_;

    double ned_1[3];
    double ned_2[3];
    bool arrow_flag = false;
    double arrow[7];

    rclcpp::Subscription<ublox_read_2::msg::RelPos>::SharedPtr sub1;
    rclcpp::Subscription<ublox_read_2::msg::RelPos>::SharedPtr sub2;

    ublox_read_2::msg::PosVelEcef ecef_msg_;
    ublox_read_2::msg::PosVelEcef base_ecef_msg_;
    ublox_read_2::msg::PosVelEcef* ecef_ptr_;

    ublox_read_2::msg::PositionVelocityTime pvt_msg_;
    ublox_read_2::msg::PositionVelocityTime base_pvt_msg_;
    ublox_read_2::msg::PositionVelocityTime* pvt_ptr_;

    ublox_read_2::msg::RelPosFlags relpos_flag_msg_;

    void cb_rov1(const ublox_read_2::msg::RelPos &msg);
    void cb_rov2(const ublox_read_2::msg::RelPos &msg);

    void initBase();
    void initRover();
    void initBrover();
    void advertiseTopics();
    void advertiseServices();


    /**
     * @brief creates a callback for the ubx message type
     * @param cls uint8_t class code -- See ubx_defs.h
     * @param type uint8_t type within class -- see ubx_defs.h
     * @param functionaddress within UBLOX_ROS
     * @param pointer to object from which the function is called
     */
    template<class M> void createCallback(uint8_t cls, uint8_t type, 
        void(ublox_ros::UBLOX_ROS::*fp)(const M &msg, uint8_t), ublox_ros::UBLOX_ROS *obj, uint8_t f9pID=0)
    {
        do
        {
            auto trampoline = [obj, fp](uint8_t _class, uint8_t _type, const ublox::UBX_message_t &ubx_msg, uint8_t f9pID=0)
            {
                (obj->*fp)(ubx_msg, f9pID);
            };

            this->ublox_->registerUBXCallback(cls, type, trampoline);
        } while (0);
    }

    constexpr double deg2rad(double x) { return M_PI/180.0 * x; }

    /**
     * @brief Slices into an iterable
     * @param iterable
     * @param xstart starting index (inclusive)
     * @param xend ending index (not inclusive)
     * @return a neatly sliced iterable [xstart, xend)
     */
    template <class T> T slice(T iteratable, int xstart, int xend) 
    {
        // Declare subvariable
        T subiterate = new T[xend-xstart];

        for(int i=xstart; i< xend; i++) 
        {
            subiterate[i-xstart] = iteratable[i];
        }
    }

    bool evalF9PID(uint8_t f9pID);
    
};

}

#endif
