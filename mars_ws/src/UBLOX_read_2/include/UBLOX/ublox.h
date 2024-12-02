#ifndef UBLOX_H
#define UBLOX_H

#include <stdint.h>
#include <iostream>
#include <fstream>
#include <algorithm>

#include "async_comm/serial.h"
#include "async_comm/udp.h"

#include "UBLOX/parsers/ubx.h"
#include "UBLOX/parsers/rtcm.h"
#include "UBLOX/parsers/nav.h"

namespace ublox
{

class UBLOX
{
public:

    typedef enum
    {
        NONE = 0,
        ROVER = 0b10, // 2
        BASE = 0b00,  //0
        BROVER = 0b01,  // 1
        RTK = 0b10,
    } rtk_type_t;
    const double PI = 3.14159265358979323846264338327950288419716939937510582097494459230781640628620;

    UBLOX(const std::string& port, int message_rate);
    ~UBLOX();

    /**
     * @brief Initializes the base to send corrections to rovers
     * @param local_host an array of strings containing the local hosts for base. (remote host for rover)
     * @param local_port an array of uint16_t that contains local port numbers for base. (remote ports for rover)
     * @param remote_host an array of strings containing the rover hosts for base (local host for rover)
     * @param remote_port an array of uint16_t that contains rover ports for base. (local ports for rover)
     * @param base_type surveyed, fixed, or moving base
     * @param rover_quantity number of rovers (number of elements in each array)
     * @param base_lat fixed base latitude, in degrees
     * @param base_lon fixed base longitude, in degrees
     * @param base_alt fixed base altitude, in meters above MSL
     * @ref Diagram:    Base(local)--------->remote(rover)
     */
    void initBase(std::string local_host[], uint16_t local_port[],
                    std::string remote_host[], uint16_t remote_port[],
                    std::string base_type, int rover_quantity, GNSS_CONSTELLATION_t constellation, int surveytime,
                    int surveyacc, double base_lat, double base_lon, double base_alt,
                    uint8_t dynamic_model=CFG_VALSET_t::DYNMODE_AIRBORNE_1G);

    void initRover(std::string local_host, uint16_t local_port,
                   std::string remote_host, uint16_t remote_port,
                   uint32_t constellation[]);

    /** 
 * @brief Initializes a rover
 * @param local_host: hostname for rover
 * @param local_port: port for rover
 * @param remote_host: hostname for base
 * @param remote_port: port for base
 *
 * Base(remote)-------------->Rover(local)
 */
     void initRover(std::string local_host, uint16_t local_port,
                    std::string remote_host, uint16_t remote_port, uint8_t dynamic_model=CFG_VALSET_t::DYNMODE_AIRBORNE_1G);

    void initRover(std::string local_host, uint16_t local_port,
                    std::string remote_host, uint16_t remote_port, GNSS_CONSTELLATION_t constellation, uint8_t dynamic_model=CFG_VALSET_t::DYNMODE_AIRBORNE_1G);

    /**
     * @brief Used to initialize a moving base that receives RTK corrections from another base.
     * @param local_host an array of strings containing the local hosts for base. (remote host for rover)
     * @param local_port an array of uint16_t that contains local port numbers for base. (remote ports for rover)
     * @param base_host an array containing only one element which is the base host for the unit
     * @param base_port an array containing only one element which is the base port for the unit
     * @param rover_host an array of strings containing the rover hosts for the brover
     * @param rover_port an array of uint16_t that contains rover ports for the brover
     * @param base_type: surveyed, fixed, or moving base
     * @param rover_quantity: number of rovers
     * @param gps 1 is on, 0 is off
     * @param glonas 1 is on, 0 is off
     * @param beidou 1 is on, 0 is off
     * @param galileo 1 is on, 0 is off
     * @ref Diagram:
            Base-------->Brover(local)--------->Rover
     */
    void initBrover(std::string local_host[], uint16_t local_port[],
                    std::string base_host[], uint16_t base_port[],
                    std::string rover_host[], uint16_t rover_port[],
                    std::string base_type, int rover_quantity, 
                    GNSS_CONSTELLATION_t constellation, uint8_t dynamic_model=CFG_VALSET_t::DYNMODE_AIRBORNE_1G);

    void initLogFile(const std::string& filename);
    void readFile(const std::string& filename);

    // Array of pointers to UDP objects.
    async_comm::UDP** udparray_ = nullptr;

    //Legacy UDP object replace by udparray_
    async_comm::UDP* udp_ = nullptr;
    async_comm::Serial serial_;

    UBX ubx_;
    RTCM rtcm_;
    NavParser nav_;

    uint8_t buffer[BUFFER_SIZE];

    std::ofstream log_file_;

    inline void registerUBXCallback(uint8_t cls, uint8_t type, UBX::ubx_cb cb)
    {
        ubx_.registerCallback(cls, type, cb);
    }
    inline void registerEphCallback(const NavParser::eph_cb& cb)
    {
        nav_.registerCallback(cb);
    }
    inline void registerGephCallback(const NavParser::geph_cb& cb)
    {
        nav_.registerCallback(cb);
    }


    rtk_type_t type_;

    void serial_read_cb(const uint8_t* buf, size_t size);
    void udp_read_cb(const uint8_t *buf, size_t size);
    void rtcm_complete_cb(const uint8_t* buf, size_t size);

    void config_gnss(bool gps, bool glonas, bool beidou, bool galileo);
    void config_gnss(GNSS_CONSTELLATION_t constellation);
    void config_f9p(uint8_t dynamic_model=CFG_VALSET_t::DYNMODE_AIRBORNE_1G);
    void config_rover();
    void config_ubx_msgs(int relpos);
    void config_rtcm_msgs(int hasRover, int baseType, int surveyacc, int surveytime, int lat, int lon, int height, int latHp, int lonHp,
                          int heightNP, GNSS_CONSTELLATION_t constellation);

    /**
     * @brief Configures the base setup by calling the correct base configuration function
     * @param base_type string moving, surveyed, or fixed
     * @param gps       0: ignore GPS 1: listen to GPS
     * @param glonas    0: ignore 1: listen
     * @param beidou    0: ignore 1: listen
     * @param galileo   0: ignore 1: listen
     * @param surveytime Time in sec for surveyin completion
     * @param surveyac Survey accuracy threshold required for surveyin completion
     */
    void config_base(std::string base_type, int gps, int glonas, int beidou,
                      int galileo, int surveytime, int surveyacc);
    void config_base_surveyed(int on_off, int gps, int glonas, int beidou,
                              int galileo, int surveytime, int surveyacc);
    /**
 * Configures moving base settings on F9P in the RAM layer (will be erased when device is rebooted)
 * 
 * @param on_off    0: all settings turned off 1: settings applied
 * @param gps       0: ignore GPS 1: listen to GPS
 * @param glonas    0: ignore 1: listen
 * @param beidou    0: ignore 1: listen
 * @param galileo   0: ignore 1: listen
 */
    void config_base_moving(int on_off, int gps, int glonas, int beidou,
                      int galileo);
    void poll_value();

    navBbrMask_t reset(uint16_t navBbrMask, uint8_t resetMode);

    /**
     * @brief Computes NED, absolute distance, pitch, and yaw
     * @param ned_1 array of length 3 NED
     * @param ned_2 array of length 3 NED
     * @return array of NED, absolute distance, roll (set to 0), pitch, yaw
     */
    void vector_math(double ned_1[], double ned_2[], double answer[]);

    CFG_VALGET_TUPLE_t CfgValGet(const CFG_VALGET_t::request_t &request);
    CFG_VALGET_TUPLE_t CfgValGet(uint32_t cfgDataKey, uint8_t layer=0, uint16_t position=0, std::string filepath="");
    CFG_VAL_DBG_t CfgValDel(uint8_t version, uint8_t layer, uint32_t cfgDataKey);
    CFG_VAL_DBG_t CfgValSet(uint8_t version, uint8_t layer, uint64_t cfgData, uint32_t cfgDataKey);

    /**
     * @brief checks version of F9P
     */
    MON_VER_t getVersion();

    /**
     * @brief Runs at the start of every ublox object initialization and checks
     * the firmware status on the F9P
     * @return true if software is up-to-date. false if update is needed
     */
    bool checkSoftware();

    uint8_t byte = 1;
    uint8_t word = 2;
    int on_off;
};
}

#endif
