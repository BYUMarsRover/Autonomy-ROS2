#ifndef UBX_H
#define UBX_H

#include <stdint.h>
#include <iostream>
#include <ctime>

#include "async_comm/serial.h"
#include "UBLOX/parsers/ubx_defs.h"

namespace ublox
{
    typedef boost::bimaps::bimap<std::string, uint32_t> bimap_type;
    typedef bimap_type::value_type value_type;

class UBX
{
public:

    UBX(async_comm::Serial& ser);
    void fill_cfg_map();

    /**
     * @brief configures settings on F9P
     */
    CFG_VAL_DBG_t configure(uint8_t version, uint8_t layer, uint64_t cfgData, uint64_t cfgDataKey);
    CFG_VALGET_TUPLE_t get_configuration(uint8_t version, uint8_t layer, uint16_t position, uint32_t cfgDataKey);
    CFG_VALGET_TUPLE_t get_configuration(uint8_t version, uint8_t layer, uint32_t cfgDataKey);
    CFG_VAL_DBG_t del_configuration(uint8_t version, uint8_t layer, uint32_t cfgDataKey);
    MON_VER_DBG_t getVersion(uint8_t attempt = 0);
    /**
     * @brief Resets the Configurations on the F9P
     * @param navBbrMask bitfield (size 2 bytes) containing areas to reset. See ubx_defs for more information
     * @param resetMode uint8_t specifying which reset mode to follow.
     */
    void reset(navBbrMask_t navBbrMask, uint8_t resetMode);

    // This function returns true when a new message has been parsed
    bool read_cb(uint8_t byte, uint8_t f9pID=0);

    // returns true if there is new data that hasn't been read
    // subsequent calls to new_data will return false after the
    // first time after a new message
    bool new_data();

    // callback handling
    typedef std::function<void(uint8_t, uint8_t, const UBX_message_t&, uint8_t)> ubx_cb;
    struct callback_t
    {
        uint8_t cls;
        uint8_t type;
        ubx_cb cb;
        uint8_t f9pID;
    };
    void registerCallback(uint8_t cls, uint8_t type, ubx_cb cb, uint8_t f9pID=0);
    std::vector<callback_t> callbacks;

    bool parsing_message();

    size_t num_messages_received();

    void set_nav_rate(uint16_t period_ms);

    void create_message(uint8_t* buffer, uint8_t msg_class, uint8_t msg_id,
                      const UBX_message_t& message, uint16_t len);

    // Send the supplied message
    bool send_message(uint8_t msg_class, uint8_t msg_id,
                      UBX_message_t& message, uint16_t len);

    void pollValue(uint8_t msg_class, uint8_t msg_id);

    // Main buffers for communication
    UBX_message_t out_message_;
    UBX_message_t in_message_;

    // low-level parsing functions
    bool decode_message(uint8_t f9pID=0);
    void calculate_checksum(const uint8_t msg_cls, const uint8_t msg_id,
                            const uint16_t len, const UBX_message_t payload,
                            uint8_t &ck_a, uint8_t &ck_b) const;

    // Translation function prior to cfgval functions
    uint32_t translate(std::string key);

    template<class T> T get_form(UBX_message_t ubx_msg, uint8_t cls, uint8_t id);

    inline double time_elapsed(clock_t start)
    {
        return ((float)(clock()-start))/CLOCKS_PER_SEC;
    }

    uint8_t cfgKeySize(const CFG_VALGET_t::response_t& cfgVal);

    // Parsing State Working Memory
    uint8_t prev_byte_;
    uint16_t buffer_head_ = 0;
    CFG_VAL_DBG_t cfgval_dbg_;
    MON_VER_DBG_t mon_ver_;
    bool start_message_ = false;
    bool end_message_ = false;
    std::vector<CFG_VALGET_t::response_t> cfg_val_get;
    parse_state_t parse_state_;
    uint8_t message_class_;
    uint8_t message_type_;
    uint16_t length_;
    uint8_t ck_a_;
    uint8_t ck_b_;
    uint32_t num_errors_ = 0;
    uint32_t num_messages_received_ = 0;
    uint8_t version;  //0 poll request, 1 poll (receiver to return config data key and value pairs)
    uint8_t layer;
    uint32_t cfgDataKey;
    uint64_t cfgData;
    uint8_t size;
    uint8_t byte = 1;
    uint8_t word = 2;

    static std::map<uint8_t, std::string> ACK_msg_map;
    static std::map<uint8_t, std::string> AID_msg_map;
    static std::map<uint8_t, std::string> CFG_msg_map;
    static std::map<uint8_t, std::string> INF_msg_map;
    static std::map<uint8_t, std::string> LOG_msg_map;
    static std::map<uint8_t, std::string> MGA_msg_map;
    static std::map<uint8_t, std::string> MON_msg_map;
    static std::map<uint8_t, std::string> NAV_msg_map;
    static std::map<uint8_t, std::string> RXM_msg_map;
    static std::map<uint8_t, std::string> SEC_msg_map;
    static std::map<uint8_t, std::string> TIM_msg_map;
    static std::map<uint8_t, std::string> UPD_msg_map;

    static std::map<uint8_t, std::map<uint8_t, std::string>> UBX_map;

    bimap_type UBX_cfg_map;
    
    // local storage
    volatile bool new_data_;

    // Serial Port
    async_comm::Serial& serial_;
};

}
#endif // UBX_H




