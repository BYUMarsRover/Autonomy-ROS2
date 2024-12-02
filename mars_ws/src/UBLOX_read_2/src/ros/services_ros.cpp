#include <UBLOX/ublox_ros.h>
#include "rclcpp/rclcpp.hpp" 

namespace ublox_ros
{
    void UBLOX_ROS::advertiseServices()
    {
        cfg_val_get_ = this->create_service<ublox_read_2::srv::CfgValGet>("CfgValGet", std::bind(&UBLOX_ROS::cfgValGet, this, std::placeholders::_1, std::placeholders::_2));
        cfg_val_get_all_ = this->create_service<ublox_read_2::srv::CfgValGetAll>("CfgValGetAll", std::bind(&UBLOX_ROS::cfgValGetAll, this, std::placeholders::_1, std::placeholders::_2));
        cfg_val_del_ = this->create_service<ublox_read_2::srv::CfgValDel>("CfgValDel", std::bind(&UBLOX_ROS::cfgValDel, this, std::placeholders::_1, std::placeholders::_2));
        cfg_val_set_ = this->create_service<ublox_read_2::srv::CfgValSet>("CfgValSet", std::bind(&UBLOX_ROS::cfgValSet, this, std::placeholders::_1, std::placeholders::_2));
        cfg_reset_ = this->create_service<ublox_read_2::srv::CfgReset>("CfgReset", std::bind(&UBLOX_ROS::cfgReset, this, std::placeholders::_1, std::placeholders::_2));
        init_module_ = this->create_service<ublox_read_2::srv::InitModule>("InitModule", std::bind(&UBLOX_ROS::initModule, this, std::placeholders::_1, std::placeholders::_2));
        get_version_ = this->create_service<ublox_read_2::srv::GetVersion>("GetVersion", std::bind(&UBLOX_ROS::getVersion, this, std::placeholders::_1, std::placeholders::_2));
    }

    void UBLOX_ROS::getVersion(const std::shared_ptr<ublox_read_2::srv::GetVersion::Request> req, std::shared_ptr<ublox_read_2::srv::GetVersion::Response> res)
    {
        ublox::MON_VER_t mon_ver = ublox_->getVersion();

        for(uint8_t i=0; i<30 && mon_ver.swVersion[i]!='\0'; i++)
        {
            res->software_version.push_back(mon_ver.swVersion[i]);
        }

        for(int i=0; i<30 && mon_ver.hwVersion[i]!='\0'; i++)
        {
            res->hardware_version.push_back(mon_ver.hwVersion[i]);
        }

        for(uint8_t i=0; i<10 && mon_ver.extension[i][0]!='\0'; i++)
        {
            std::string extend;
            for(uint8_t j=0; j<30 && mon_ver.extension[i][j]!='\0'; j++)
            {
                std::cerr<<mon_ver.extension[i][j];
                extend.push_back(mon_ver.extension[i][j]);
            }
            res->extension.push_back(extend);
        }
    }

    // void add(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request, std::shared_ptr<example_interfaces::srv::AddTwoInts::Response>      response)
    // {
    // response->sum = request->a + request->b;
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld",
    //                 request->a, request->b);
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
    // }
    
    void UBLOX_ROS::cfgValGet(const std::shared_ptr<ublox_read_2::srv::CfgValGet::Request> req, std::shared_ptr<ublox_read_2::srv::CfgValGet::Response> res)
    {
        ublox::CFG_VALGET_TUPLE_t response = ublox_->CfgValGet(req->key, req->layer, req->position, req->filepath);
        std::vector<ublox::CFG_VALGET_t::response_t> cfgVector_ublox = std::get<1>(response);
        for(int i=0; i<cfgVector_ublox.size(); i++)
        {
            ublox_read_2::msg::CfgValGetType cfg_ros;
            cfg_ros.version = cfgVector_ublox[i].version;
            cfg_ros.layer = cfgVector_ublox[i].layer;
            cfg_ros.position = cfgVector_ublox[i].position.position;
            cfg_ros.key_id = cfgVector_ublox[i].cfgDataKey.keyID;
            cfg_ros.keyname = std::string(cfgVector_ublox[i].keyName);
            cfg_ros.data = cfgVector_ublox[i].cfgData.data;
            
            res->cfg_data.push_back(cfg_ros);
        }
        res->ack=std::get<0>(response).got_ack;
        res->nack=std::get<0>(response).got_nack;
        res->gotcfg=std::get<0>(response).got_cfg_val;
        res->flags=std::get<0>(response).flags;
    }

    void UBLOX_ROS::cfgValGetAll(const std::shared_ptr<ublox_read_2::srv::CfgValGetAll::Request> req, std::shared_ptr<ublox_read_2::srv::CfgValGetAll::Response> res)
    {
        ublox::CFG_VALGET_TUPLE_t response = ublox_->CfgValGet(0x0fff0000, req->layer, req->position, req->filepath);
        std::vector<ublox::CFG_VALGET_t::response_t> cfgVector_ublox = std::get<1>(response);
        for(int i=0; i<cfgVector_ublox.size(); i++)
        {
            ublox_read_2::msg::CfgValGetType cfg_ros;
            cfg_ros.version = cfgVector_ublox[i].version;
            cfg_ros.layer = cfgVector_ublox[i].layer;
            cfg_ros.position = cfgVector_ublox[i].position.position;
            cfg_ros.key_id = cfgVector_ublox[i].cfgDataKey.keyID;
            cfg_ros.keyname = std::string(cfgVector_ublox[i].keyName);
            cfg_ros.data = cfgVector_ublox[i].cfgData.data;
            
            res->cfg_data.push_back(cfg_ros);
        }
        res->ack=std::get<0>(response).got_ack;
        res->nack=std::get<0>(response).got_nack;
        res->gotcfg=std::get<0>(response).got_cfg_val;
        res->flags=std::get<0>(response).flags;
    }

    void UBLOX_ROS::cfgValDel(const std::shared_ptr<ublox_read_2::srv::CfgValDel::Request> req, std::shared_ptr<ublox_read_2::srv::CfgValDel::Response> res)
    {

        ublox::CFG_VAL_DBG_t response = ublox_->CfgValDel(0, req->layer, req->key);

        res->got_ack = response.got_ack;
        res->got_nack = response.got_nack;
    }



    void UBLOX_ROS::cfgValSet(const std::shared_ptr<ublox_read_2::srv::CfgValSet::Request> req, std::shared_ptr<ublox_read_2::srv::CfgValSet::Response> res)
    {
        ublox::CFG_VAL_DBG_t response = ublox_->CfgValSet(0, req->layer, req->cfg_data, req->key);

        res->got_ack = response.got_ack;
        res->got_nack = response.got_nack;
    }

    void UBLOX_ROS::cfgReset(const std::shared_ptr<ublox_read_2::srv::CfgReset::Request> req, std::shared_ptr<ublox_read_2::srv::CfgReset::Response> res)
    {
        ublox::navBbrMask_t bitfield =  ublox_->reset(req->nav_bbr_mask, req->reset_mode);

        // std::cerr<<"eph: "<< bitfield.eph<<std::endl;

        res->eph = bitfield.eph;
        res->alm = bitfield.alm;
        res->health = bitfield.health;
        res->klob = bitfield.klob;
        res->pos = bitfield.pos;
        res->clkd = bitfield.clkd;
        res->osc = bitfield.osc;
        res->utc = bitfield.utc;
        res->rtc = bitfield.rtc;
        res->aop = bitfield.aop;
    }

    void UBLOX_ROS::initModule(const std::shared_ptr<ublox_read_2::srv::InitModule::Request> req, std::shared_ptr<ublox_read_2::srv::InitModule::Response> res)
    {
        switch(req->type)
        {
            case 0:
                initBase();
                break;
            case 1:
                initRover();
                break;
            case 2:
                initBrover();
                break;
            default:
                std::cerr<<"Error: initModule invalid type\n";
                return;
                break;
        }
    }

}