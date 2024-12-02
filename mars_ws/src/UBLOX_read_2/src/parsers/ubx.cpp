#include <chrono>
#include <stdio.h>

#include "async_comm/udp.h"

#include "UBLOX/parsers/ubx.h"

using namespace std::chrono;
using namespace std;

#define DEG2RAD (3.14159 / 180.0)
// #ifndef NDEBUG
#define DBG(...) fprintf(stderr, __VA_ARGS__)
// #define DBG(...) //fprintf(stderr, __VA_ARGS__)
// #define DBG(...) fprintf(stderr, )
// #else
// #define DBG(...)
// #endif

namespace ublox
{

UBX::UBX(async_comm::Serial& ser) :
    serial_(ser)
{
    buffer_head_ = 0;
    parse_state_ = START;
    message_class_ = 0;
    message_type_ = 0;
    length_ = 0;
    ck_a_ = 0;
    ck_b_ = 0;
    prev_byte_ = 0;
    start_message_ = false;
    new_data_ = false;
    end_message_ = false;
    memset(&cfgval_dbg_, 0, sizeof(CFG_VAL_DBG_t));

    fill_cfg_map();
}

bool UBX::parsing_message()
{
    return (start_message_ == true && end_message_ == false);
}

bool UBX::new_data()
{
    bool tmp = new_data_;
    new_data_ = false;
    return tmp;
}

bool UBX::read_cb(uint8_t byte, uint8_t f9pID)
{
    switch (parse_state_)
    {
    case START:
        if (byte == START_BYTE_2 && prev_byte_ == START_BYTE_1)
        {
            buffer_head_ = 0;
            parse_state_ = GOT_START_FRAME;
            message_class_ = 0;
            message_type_ = 0;
            length_ = 0;
            ck_a_ = 0;
            ck_b_ = 0;
            start_message_ = true;
            end_message_ = false;
            memset(&in_message_, 0, sizeof(in_message_));
        }
        break;
    case GOT_START_FRAME:
        message_class_ = byte;
        parse_state_ = GOT_CLASS;
        break;
    case GOT_CLASS:
        message_type_ = byte;
        parse_state_ = GOT_MSG_ID;
        break;
    case GOT_MSG_ID:
        length_ = byte;
        parse_state_ = GOT_LENGTH1;
        break;
    case GOT_LENGTH1:
        length_ |= (uint16_t) byte << 8;
        parse_state_ = GOT_LENGTH2;
        if (length_ > BUFFER_SIZE)
        {
            num_errors_++;
            parse_state_ = START;
            prev_byte_ = byte;
            end_message_ = false;
            start_message_ = false;
            return false;
        }
        break;
    case GOT_LENGTH2:
        if (buffer_head_ < length_)
        {
            // push the byte onto the data buffer
            in_message_.buffer[buffer_head_] = byte;
            if (buffer_head_ == length_-1)
            {
                parse_state_ = GOT_PAYLOAD;
            }
            buffer_head_++;
        }
        break;
    case GOT_PAYLOAD:
        ck_a_ = byte;
        parse_state_ = GOT_CK_A;
        break;
    case GOT_CK_A:
        ck_b_ = byte;
        parse_state_ = GOT_CK_B;
        break;
    default:
        num_errors_++;
        parse_state_ = START;
        end_message_ = false;
        start_message_ = false;
        break;
    }

    // if(f9pID==1) DBG("Parse State: %i\n", parse_state_);
    // If we have a complete packet, then try to parse it
    if (parse_state_ == GOT_CK_B)
    {
        if (decode_message(f9pID))
        {
            parse_state_ = START;
            end_message_ = true;
            start_message_ = false;
            new_data_ = true;
            prev_byte_ = byte;
            return true;
        }
        else
        {
            // indicate error if it didn't work
            DBG("\nFailed to parse message, f9pID: %i, ParseState: %i, CLASS_ID: %i, MSG_ID: %i\n", f9pID, parse_state_, message_class_, message_type_);
            num_errors_++;
            parse_state_ = START;
            start_message_ = false;
            end_message_ = false;
        }
    }
    prev_byte_ = byte;
    return false;
}

/*  decode_message()
    Called by read_cb
    Returns true if the checksum is successful. Otherwise returns false.

*/
bool UBX::decode_message(uint8_t f9pID)
{
    if(f9pID==1)
    {
        // DBG("Decoding baseveldata\n");
    }
    // First, check the checksum
    uint8_t ck_a, ck_b;
    calculate_checksum(message_class_, message_type_, length_, in_message_, ck_a, ck_b);
    if (ck_a != ck_a_ || ck_b != ck_b_)
    {
        if(f9pID==1)
        {
            DBG("Returning false because checksums did not match!\n");
            DBG("Message cka: %i, calculated cka: %i\n", ck_a_, ck_a);
            DBG("Message ckb: %i, calculated ckb: %i\n", ck_b_, ck_b);
        }
        return false;
    }
    uint8_t version; //0 poll request, 1 poll (receiver to return config data key and value pairs)
    uint8_t layer;
    uint8_t reserved1[2];
    uint32_t cfgDataKey;
    uint64_t cfgData;
    num_messages_received_++;

    // Parse the payload
    switch (message_class_)
    {
    case CLASS_ACK:
        switch (message_type_)
        {
        case ACK_ACK:
            if(in_message_.ACK_ACK.clsID==CLASS_CFG)
                cfgval_dbg_.got_ack = true;
            DBG("ACK_ACK: %s\n", UBX_map[in_message_.ACK_ACK.clsID][in_message_.ACK_ACK.msgID].c_str());
            break;
        case ACK_NACK:
            if(in_message_.ACK_NACK.clsID==CLASS_CFG)
                cfgval_dbg_.got_nack = true;
            DBG("ACK_NACK: %s\n", UBX_map[in_message_.ACK_ACK.clsID][in_message_.ACK_ACK.msgID].c_str());
            break;
        default:
            DBG("%d\n", message_type_);
            break;
        }
        break;
   case CLASS_CFG: //only needed for getting data
       switch (message_type_)
       {
       case CFG_VALGET:
       {
            cfg_val_get.clear();
            
            DBG("CFG_VALGET\n");
            uint8_t version = in_message_.buffer[0];
            // DBG("Version: %i\n", version);
            uint8_t layer = in_message_.buffer[1];
            // DBG("Layer: %i\n", layer);
            CFG_VALGET_t::position_t position;
            position.buffer[0] = in_message_.buffer[2];
            position.buffer[1] = in_message_.buffer[3];
            // DBG("Position: %i\n", position);

            // DBG("Length: %i\n", length_);

            for(int bufIndex = 4; bufIndex<length_; )
            {
                CFG_VALGET_t::response_t cfgVal;
                memset(&cfgVal, 0, sizeof(CFG_VALGET_t::response_t));
                cfgVal.version = version;
                cfgVal.layer = layer;
                cfgVal.position = position;
                for(int keyIndex = bufIndex; keyIndex<bufIndex+4; keyIndex++)
                {
                    cfgVal.cfgDataKey.buffer[keyIndex-bufIndex] = in_message_.buffer[keyIndex];
                }
                bufIndex = bufIndex+4;
                // DBG("Key: %i, messageID: %i, groupID: %i, Size: %i\n", cfgVal.cfgDataKey.keyID, cfgVal.cfgDataKey.msgID, cfgVal.cfgDataKey.groupID, cfgKeySize(cfgVal));
                for(int dataIndex = bufIndex; dataIndex < bufIndex+cfgKeySize(cfgVal); dataIndex++)
                {
                    cfgVal.cfgData.buffer[dataIndex-bufIndex] = in_message_.buffer[dataIndex];
                }
                // DBG("Value: %i\n", cfgVal.cfgData.data);
                if(UBX_cfg_map.right.count(cfgVal.cfgDataKey.keyID)!=0)
                {
                    cfgVal.keyName = UBX_cfg_map.right.at(cfgVal.cfgDataKey.keyID);
                }
                else
                {
                    cfgVal.keyName = "No keyName found";
                }
                // DBG("Cfg Map: %s\n", cfgVal.keyName.c_str());
                cfg_val_get.push_back(cfgVal);
                bufIndex = bufIndex+cfgKeySize(cfgVal);
            }
            
            cfgval_dbg_.got_cfg_val=true;
            break;
       }
       case CFG_VALDEL:
            DBG("CFG_VALDEL: \n");
            DBG("Key: %i \n", in_message_.CFG_VALDEL.cfgDataKey.keyID);
            cfgval_dbg_.got_cfg_val=true;
            break;
       default:
           DBG("unknown: %x\n", message_type_);
           break;
       }
    case CLASS_MON:
        switch(message_type_)
        {
            case MON_VER:
                // DBG("MON_VER of length: %i\n", length_);
                mon_ver_.got_mon = true;
                mon_ver_.mon_ver = in_message_.MON_VER;
        }
    default:
        // DBG((UBX_map[message_class_][message_type_]+"\n").c_str());
        break;
    }

    if(f9pID==1)
    {
        // DBG("Looking for callbacks of class: %i, type: %i\n", message_class_, message_type_);
    }
    // call callbacks
    for (auto& cb : callbacks)
    {
        if (message_class_ == cb.cls && message_type_ == cb.type)
        {
            if(f9pID==1)
            {
                // DBG("Calling class: %i, type: %i\n", cb.cls, cb.type);
            }
            cb.cb(message_class_, message_type_, in_message_, f9pID);
        }
    }

    new_data_ = true;
    return true;
}

void UBX::registerCallback(uint8_t cls, uint8_t type,
                ubx_cb cb, uint8_t f9pID)
{
    callbacks.push_back({cls, type, cb, f9pID});
}

void UBX::calculate_checksum(const uint8_t msg_cls, const uint8_t msg_id, const uint16_t len, const UBX_message_t payload, uint8_t& ck_a, uint8_t& ck_b) const
{
    if (msg_cls == 5)
        volatile int debug =1;
    ck_a = ck_b = 0;

    // Add in class
    ck_a += msg_cls;
    ck_b += ck_a;

    // Id
    ck_a += msg_id;
    ck_b += ck_a;

    // Length
    ck_a += len & 0xFF;
    ck_b += ck_a;
    ck_a += (len >> 8) & 0xFF;
    ck_b += ck_a;

    // Payload
    for (int i = 0; i < len; i ++)
    {
        ck_a += payload.buffer[i];
        ck_b += ck_a;
    }
}

void UBX::create_message(uint8_t* buffer, uint8_t msg_class, uint8_t msg_id, const UBX_message_t& message, uint16_t len)
{
    memset(buffer, 0, len);
    buffer[0] = START_BYTE_1;
    buffer[1] = START_BYTE_2;
    buffer[2] = msg_class;
    buffer[3] =  msg_id;
    buffer[4] = len & 0xFF;
    buffer[5] = (len >> 8) & 0xFF;
    for(int i=0; i<len; i++)
    {
        buffer[i+6] = message.buffer[i];
    }

    uint8_t ck_a, ck_b;
    calculate_checksum(msg_class, msg_id, len, message, ck_a, ck_b);
    buffer[6+len] = ck_a;
    buffer[7+len] = ck_b;
    // DBG("Create message cka: %i\n", ck_a);
    // DBG("Create message ckb: %i\n", ck_b);
}

// Sending messages to the f9p
// These messages are either CFG_VALSET, CFG_VALGET, or CFG_VALDEL
// Returns true if successfully send the message to the f9p module
bool UBX::send_message(uint8_t msg_class, uint8_t msg_id, UBX_message_t& message, uint16_t len)
{    
    // First, calculate the checksum
    uint8_t ck_a, ck_b;
    calculate_checksum(msg_class, msg_id, len, message, ck_a, ck_b);

    // Send message
    serial_.send_byte(START_BYTE_1);
    serial_.send_byte(START_BYTE_2);
    serial_.send_byte(msg_class);
    serial_.send_byte(msg_id);
    serial_.send_byte(len & 0xFF);
    serial_.send_byte((len >> 8) & 0xFF);
    serial_.send_bytes(message.buffer, len);
    serial_.send_byte(ck_a);
    serial_.send_byte(ck_b);
    return true;
}

void UBX::pollValue(uint8_t msg_class, uint8_t msg_id)
{
    send_message(msg_class, msg_id, out_message_, 0);
}

void UBX::set_nav_rate(uint16_t message_rate)
{
    fprintf(stderr, "Setting message rate to %d hz\n", message_rate);

    uint16_t period_ms = uint16_t(1000)/message_rate;

    DBG("period_ms: %d\n", period_ms);

    configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, period_ms, CFG_VALSET_t::RATE_MEAS);
    configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1, CFG_VALSET_t::RATE_NAV);
    DBG("nav_rate: %d\n", 1);
    configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, CFG_VALSET_t::TIME_REF_UTC, CFG_VALSET_t::RATE_TIMEREF);
}

CFG_VAL_DBG_t UBX::configure(uint8_t version, uint8_t layer, uint64_t cfgData, uint64_t cfgDataKey)
{
    memset(&out_message_, 0, sizeof(CFG_VALSET_t));
    memset(&cfgval_dbg_, 0, sizeof(CFG_VAL_DBG_t));
    out_message_.CFG_VALSET.version = version;
    out_message_.CFG_VALSET.layer = layer;
    out_message_.CFG_VALSET.cfgDataKey.keyID = cfgDataKey;
    uint8_t cfgSize = out_message_.CFG_VALSET.cfgDataKey.size;
    uint8_t cfgmsgSize = sizeof(CFG_VALSET_t::version)+sizeof(CFG_VALSET_t::reserved1)+sizeof(CFG_VALSET_t::layer)+sizeof(CFG_VALSET_t::cfgDataKey);
    switch (cfgSize)
    {
        case CFG_KEY_ID_t::SIZE_ONE_BIT:
            out_message_.CFG_VALSET.cfgData.bytes[0] = cfgData;
            cfgmsgSize = cfgmsgSize+1;
            break;
        case CFG_KEY_ID_t::SIZE_ONE_BYTE:
            out_message_.CFG_VALSET.cfgData.bytes[0] = cfgData;
            cfgmsgSize = cfgmsgSize+1;
            break;
        case CFG_KEY_ID_t::SIZE_TWO_BYTE:
            out_message_.CFG_VALSET.cfgData.half_word[0] = cfgData;
            cfgmsgSize = cfgmsgSize+2;
            break;
        case CFG_KEY_ID_t::SIZE_FOUR_BYTE:
            out_message_.CFG_VALSET.cfgData.word[0] = cfgData;
            cfgmsgSize = cfgmsgSize+4;
            break;
        case CFG_KEY_ID_t::SIZE_EIGHT_BYTE:
            out_message_.CFG_VALSET.cfgData.two_word = cfgData;
            cfgmsgSize = cfgmsgSize+8;
            break;
        default:
            DBG("Unknown Size: %i. Configuration will not sent\n", cfgSize);
            cfgmsgSize = 0;
        break;
    }
    send_message(CLASS_CFG, CFG_VALSET, out_message_, cfgmsgSize);
    // std::cerr<<"Configured "<< cfgDataKey<<" to "<<cfgData<<std::endl;

    clock_t start = clock();

    while( !cfgval_dbg_.got_ack && !cfgval_dbg_.got_nack && time_elapsed(start) < 5);

    return cfgval_dbg_;
}

CFG_VALGET_TUPLE_t UBX::get_configuration(uint8_t version, uint8_t layer, uint16_t position, uint32_t cfgDataKey)
{
    //DBG("%s\n", (UBX_cfg_map.right.find(cfgDataKey)->second).c_str());
    memset(&out_message_, 0, sizeof(CFG_VALGET_t::request_t));
    memset(&cfgval_dbg_, 0, sizeof(CFG_VAL_DBG_t));
    out_message_.CFG_VALGET.version = version;
    out_message_.CFG_VALGET.layer = layer;
    out_message_.CFG_VALGET.position = position;
    out_message_.CFG_VALGET.cfgDataKey.keyID = cfgDataKey;
    send_message(CLASS_CFG, CFG_VALGET, out_message_, sizeof(CFG_VALGET_t::request_t));
    //std::cerr<<"Got configuration of "<<cfgDataKey<<" to "<<cfgData<<std::endl;

    clock_t start = clock();

    while( (!(cfgval_dbg_.got_ack && cfgval_dbg_.got_cfg_val) && !cfgval_dbg_.got_nack) && time_elapsed(start) < 5);

    return {cfgval_dbg_, cfg_val_get};

}

CFG_VALGET_TUPLE_t UBX::get_configuration(uint8_t version, uint8_t layer, uint32_t cfgDataKey)
{
    //DBG("%s\n", (UBX_cfg_map.right.find(cfgDataKey)->second).c_str());
    memset(&out_message_, 0, sizeof(CFG_VALGET_t::request_t));
    memset(&cfgval_dbg_, 0, sizeof(CFG_VAL_DBG_t));
    out_message_.CFG_VALGET.version = version;
    out_message_.CFG_VALGET.layer = layer;
    out_message_.CFG_VALGET.cfgDataKey.keyID = cfgDataKey;
    send_message(CLASS_CFG, CFG_VALGET, out_message_, sizeof(CFG_VALGET_t::request_t));
    //std::cerr<<"Got configuration of "<<cfgDataKey<<" to "<<cfgData<<std::endl;

    clock_t start = clock();

    while( (!(cfgval_dbg_.got_ack && cfgval_dbg_.got_cfg_val) && !cfgval_dbg_.got_nack) && time_elapsed(start) < 5);

    return {cfgval_dbg_, cfg_val_get};

}

//Deletes configuration values specified by the key
CFG_VAL_DBG_t UBX::del_configuration(uint8_t version, uint8_t layer, uint32_t cfgDataKey)
{
    memset(&out_message_, 0, sizeof(CFG_VALDEL_t));
    memset(&cfgval_dbg_, 0, sizeof(CFG_VAL_DBG_t));
    out_message_.CFG_VALDEL.version = version;
    out_message_.CFG_VALDEL.layer = layer;
    out_message_.CFG_VALDEL.cfgDataKey.keyID = cfgDataKey;
    send_message(CLASS_CFG, CFG_VALDEL, out_message_, sizeof(CFG_VALDEL_t));
    // std::cerr<<"Deleted configuration of "<<cfgDataKey<<std::endl;

    clock_t start = clock();

    while( !cfgval_dbg_.got_ack && !cfgval_dbg_.got_nack && time_elapsed(start) < 5);

    return cfgval_dbg_;
}

MON_VER_DBG_t UBX::getVersion(uint8_t attempt)
{
    mon_ver_.got_mon = false;
    
    pollValue(CLASS_MON, MON_VER);
    pollValue(CLASS_MON, MON_VER);

    clock_t start = clock();

    while(!mon_ver_.got_mon && time_elapsed(start) < 2);
    if(mon_ver_.got_mon)
    {
        return mon_ver_;
    }
    else if(attempt<2) //Try again
    {
        return getVersion(attempt+1);
    }
    else    //Time out
    {
        DBG("GET SOFTWARE TIMED OUT\n");
        return mon_ver_;
    }
    
}

void UBX::reset(navBbrMask_t navBbrMask, uint8_t resetMode)
{
    memset(&out_message_, 0, sizeof(CFG_RST_t));
    out_message_.CFG_RST.navBbrMask=navBbrMask;
    out_message_.CFG_RST.resetMode=resetMode;
    send_message(CLASS_CFG, CFG_RST, out_message_, sizeof(CFG_RST_t));
    DBG("Reset Successful\n");
}

uint32_t UBX::translate(std::string key)
{
    std::string::size_type leftover;

    uint32_t numkey = std::stoi(key, &leftover, 0);

    return numkey;
}

uint8_t UBX::cfgKeySize(const CFG_VALGET_t::response_t& cfgVal)
    {
        switch(cfgVal.cfgDataKey.size)
        {
            case CFG_KEY_ID_t::SIZE_ONE_BIT:
                // DBG("One bit\n");
                return 1;
                break;
            case CFG_KEY_ID_t::SIZE_ONE_BYTE:
                // DBG("One byte\n");
                return 1;
                break;
            case CFG_KEY_ID_t::SIZE_TWO_BYTE:
                // DBG("Two bytes\n");
                return 2;
                break;
            case CFG_KEY_ID_t::SIZE_FOUR_BYTE:
                // DBG("Four bytes\n");
                return 4;
                break;
            case CFG_KEY_ID_t::SIZE_EIGHT_BYTE:
                // DBG("Eight bytes\n");
                return 8;
                break;
            default:
                DBG("Error: Unknown size: %i\n", cfgVal.cfgDataKey.size);
                return 0;
                break;
        }
    }

}
