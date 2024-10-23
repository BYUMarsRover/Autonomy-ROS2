#include <UBLOX/ublox_ros.h>

namespace ublox_ros
{
    void UBLOX_ROS::advertiseTopics()
    {
        pvt_pub_ = this->create_publisher<ublox_read_2::msg::PositionVelocityTime>("PosVelTime", 10);
        relpos_pub_ = this->create_publisher<ublox_read_2::msg::RelPos>("RelPos", 10);
        relposflag_pub_ = this->create_publisher<ublox_read_2::msg::RelPosFlags>("RelPosFlags", 10);
        ecef_pub_ = this->create_publisher<ublox_read_2::msg::PosVelEcef>("PosVelEcef", 10);
        survey_status_pub_ = this->create_publisher<ublox_read_2::msg::SurveyStatus>("SurveyStatus", 10);
        eph_pub_ = this->create_publisher<ublox_read_2::msg::Ephemeris>("Ephemeris", 10);
        geph_pub_ = this->create_publisher<ublox_read_2::msg::GlonassEphemeris>("GlonassEphemeris", 10);
        obs_pub_ = this->create_publisher<ublox_read_2::msg::ObsVec>("Obs", 10);
        base_ecef_pub_ = this->create_publisher<ublox_read_2::msg::PosVelEcef>("base/PosVelEcef", 10);
        base_pvt_pub_ = this->create_publisher<ublox_read_2::msg::PositionVelocityTime>("base/PosVelTime", 10);
        rtcm_input_pub_ = this->create_publisher<ublox_read_2::msg::RTCMInput>("RTCMInput", 10);
        sat_status_pub_ = this->create_publisher<ublox_read_2::msg::SatelliteStatus>("SatelliteStatus", 10);
        // nav_sat_fix_pub_ = this->create_publisher<sensor_msgs::NavSatFix>("NavSatFix");
        // nav_sat_status_pub_ = this->create_publisher<sensor_msgs::NavSatStatus>("NavSatStatus");
    }
    
    // Callback function for subscriber to RelPos for a given RelPos message.
// NOTE: This message is not the same as ublox::NAV_RELPOSNED_t, since that one
// deals with messages from the f9p
void UBLOX_ROS::cb_rov1(const ublox_read_2::msg::RelPos &msg) {
    ned_1[0] = msg.rel_pos_ned[0];  //North
    ned_1[1] = msg.rel_pos_ned[1];  //East
    ned_1[2] = msg.rel_pos_ned[2];  //Down
}

// Callback function for subscriber to second RelPos.
// NOTE: This message is not the same as ublox::NAV_RELPOSNED_t, since that one
// deals with messages from the f9p
void UBLOX_ROS::cb_rov2(const ublox_read_2::msg::RelPos &msg) {
    ned_2[0] = msg.rel_pos_ned[0];  //North
    ned_2[1] = msg.rel_pos_ned[1];  //East
    ned_2[2] = msg.rel_pos_ned[2];  //Down
}

void UBLOX_ROS::pvtCB(const ublox::UBX_message_t &ubx_msg, uint8_t f9pID)
{
    ublox::NAV_PVT_t msg = ubx_msg.NAV_PVT;

    // RCLCPP_INFO_STREAM(this->get_logger(), "Callback: pvtCB. ID: " << int(f9pID));

    if(!evalF9PID(f9pID)) return;

    *pvt_tow_ptr_ = msg.iTOW;
    // out.iTOW = msg.iTow;
    pvt_ptr_->header.stamp = this->now(); ///TODO: Do this right
    pvt_ptr_->year = msg.year;
    pvt_ptr_->month = msg.month;
    pvt_ptr_->day = msg.day;
    pvt_ptr_->hour = msg.hour;
    pvt_ptr_->min = msg.min;
    pvt_ptr_->sec = msg.sec;
    pvt_ptr_->nano = msg.nano;
    pvt_ptr_->t_acc = msg.tAcc;
    pvt_ptr_->valid = msg.valid;
    pvt_ptr_->fix_type = msg.fixType;
    pvt_ptr_->flags = msg.flags;
    pvt_ptr_->flags2 = msg.flags2;
    pvt_ptr_->num_sv = msg.numSV;
    pvt_ptr_->lla[0] = msg.lat*1e-7;
    pvt_ptr_->lla[1] = msg.lon*1e-7;
    pvt_ptr_->lla[2] = msg.height*1e-3;
    pvt_ptr_->h_msl = msg.hMSL*1e-3;
    pvt_ptr_->h_acc = msg.hAcc*1e-3;
    pvt_ptr_->v_acc = msg.vAcc*1e-3;
    pvt_ptr_->vel_ned[0] = msg.velN*1e-3;
    pvt_ptr_->vel_ned[1] = msg.velE*1e-3;
    pvt_ptr_->vel_ned[2] = msg.velD*1e-3;
    pvt_ptr_->g_speed = msg.gSpeed*1e-3;
    pvt_ptr_->head_mot = msg.headMot*1e-5;
    pvt_ptr_->s_acc = msg.sAcc*1e-3;
    pvt_ptr_->head_acc = msg.headAcc*1e-5;
    pvt_ptr_->p_dop = msg.pDOP*0.01;
    pvt_ptr_->head_veh = msg.headVeh*1e-5;

    pvt_pub_->publish(*pvt_ptr_);

    ecef_ptr_->header.stamp = this->now();
    ecef_ptr_->fix = pvt_ptr_->fix_type;
    ecef_ptr_->lla[0] = pvt_ptr_->lla[0];
    ecef_ptr_->lla[1] = pvt_ptr_->lla[1];
    ecef_ptr_->lla[2] = pvt_ptr_->lla[2];
    ecef_ptr_->horizontal_accuracy = pvt_ptr_->h_acc;
    ecef_ptr_->vertical_accuracy = pvt_ptr_->v_acc;
    ecef_ptr_->speed_accuracy = pvt_ptr_->s_acc;

    if (*ecef_pos_tow_ptr_ == *pvt_tow_ptr_ && 
        *ecef_pos_tow_ptr_ == *ecef_vel_tow_ptr_)
    {
        ecef_pub_->publish(*ecef_ptr_);
    }
}


void UBLOX_ROS::relposCB(const ublox::UBX_message_t &ubx_msg, uint8_t f9pID)
{
    // std::cerr<<"relposCB"<<std::endl;
    
    ublox::NAV_RELPOSNED_t msg = ubx_msg.NAV_RELPOSNED;
    
    // Create the message to be outputted
    ublox_read_2::msg::RelPos out;


    // out.iTOW = msg.iTow*1e-3;
    out.header.stamp = this->now(); /// TODO: do this right
    out.ref_station_id = msg.refStationId;
    out.rel_pos_ned[0] = msg.relPosN*1e-2;
    out.rel_pos_ned[1] = msg.relPosE*1e-2;
    out.rel_pos_ned[2] = msg.relPosD*1e-2;
    out.rel_pos_length = msg.relPosLength*1e-2;
    out.rel_pos_heading = deg2rad(msg.relPosHeading*1e-5);
    out.rel_pos_hp_ned[0] = msg.relPosHPN*1e-3*.1;
    out.rel_pos_hp_ned[1] = msg.relPosHPE*1e-3*.1;
    out.rel_pos_hp_ned[2] = msg.relPosHPD*1e-3*.1;
    out.rel_pos_hp_length = msg.relPosHPLength*1e-3*.1;
    out.acc_ned[0] = msg.accN*1e-3*.1;
    out.acc_ned[1] = msg.accE*1e-3*.1;
    out.acc_ned[2] = msg.accD*1e-3*.1;
    out.acc_length = msg.accLength*1e-3*.1;
    out.acc_heading = deg2rad(msg.accHeading*1e-5);
    out.flags = msg.flags.all_flags;


    relpos_flag_msg_.header.stamp = out.header.stamp;
    relpos_flag_msg_.gnss_fix_ok = msg.flags.gnssFixOk;
    relpos_flag_msg_.diff_soln = msg.flags.diffSoln;
    relpos_flag_msg_.rel_pos_valid = msg.flags.relPosValid;
    relpos_flag_msg_.float_carr_soln = msg.flags.floatCarrSoln;
    relpos_flag_msg_.fixed_carr_soln = msg.flags.fixedCarrSoln;
    relpos_flag_msg_.is_moving = msg.flags.isMoving;
    relpos_flag_msg_.ref_pos_miss = msg.flags.refPosMiss;
    relpos_flag_msg_.ref_obs_miss = msg.flags.refObsMiss;
    relpos_flag_msg_.rel_pos_heading_valid = msg.flags.relPosHeadingValid;
    relpos_flag_msg_.rel_pos_normalized = msg.flags.relPosNormalized;
    relpos_flag_msg_.flags = msg.flags.all_flags;

    if (arrow_flag == true) {

    // Perform vector_math and assign values to arrow. (see ublox_ros.h for
    // variable declarations)
    ublox_->vector_math(ned_1, ned_2, arrow);

    // Assign all the values
    out.arrow_ned[0] = arrow[0];
    out.arrow_ned[1] = arrow[1];
    out.arrow_ned[2] = arrow[2];
    out.arrow_length = arrow[3];
    out.arrow_rpy[0] = arrow[4];
    out.arrow_rpy[1] = arrow[5];
    out.arrow_rpy[2] = arrow[6];
  }
    // Publish the RelPos ROS message
    relpos_pub_->publish(out);
    relposflag_pub_->publish(relpos_flag_msg_);
}

void UBLOX_ROS::svinCB(const ublox::UBX_message_t &ubx_msg, uint8_t f9pID)
{
    ublox::NAV_SVIN_t msg = ubx_msg.NAV_SVIN;
    ublox_read_2::msg::SurveyStatus out;
    out.header.stamp = this->now(); /// TODO: do this right
    out.dur = msg.dur;
    out.mean_xyz[0] = msg.meanX*1e-2;
    out.mean_xyz[1] = msg.meanY*1e-2;
    out.mean_xyz[2] = msg.meanZ*1e-2;
    out.mean_xyz_hp[0] = msg.meanXHP*1e-3;
    out.mean_xyz_hp[1] = msg.meanYHP*1e-3;
    out.mean_xyz_hp[2] = msg.meanZHP*1e-3;
    out.mean_acc = msg.meanAcc;
    out.obs = msg.obs;
    out.valid = msg.valid;
    out.active = msg.active;
    survey_status_pub_->publish(out);

}

void UBLOX_ROS::posECEFCB(const ublox::UBX_message_t &ubx_msg, uint8_t f9pID)
{
    ublox::NAV_POSECEF_t msg = ubx_msg.NAV_POSECEF;

    // RCLCPP_INFO_STREAM(this->get_logger(), "Callback: posECEFCB. ID: " << int(f9pID));

    if(!evalF9PID(f9pID)) return;

    *ecef_pos_tow_ptr_ = msg.iTOW;
    ecef_ptr_->header.stamp = this->now();
    ecef_ptr_->position[0] = msg.ecefX*1e-2;
    ecef_ptr_->position[1] = msg.ecefY*1e-2;
    ecef_ptr_->position[2] = msg.ecefZ*1e-2;

    if (*ecef_pos_tow_ptr_ == *pvt_tow_ptr_ && 
        *ecef_pos_tow_ptr_ == *ecef_vel_tow_ptr_)
    {
        ecef_pub_->publish(*ecef_ptr_);
    }

}

void UBLOX_ROS::velECEFCB(const ublox::UBX_message_t &ubx_msg, uint8_t f9pID)
{
    ublox::NAV_VELECEF_t msg = ubx_msg.NAV_VELECEF;

    // RCLCPP_INFO_STREAM(this->get_logger(), "Callback: velECEFCB. ID: " << int(f9pID));

    if(!evalF9PID(f9pID)) return;

    *ecef_vel_tow_ptr_ = msg.iTOW;
    ecef_ptr_->header.stamp = this->now();
    ecef_ptr_->velocity[0] = msg.ecefVX*1e-2;
    ecef_ptr_->velocity[1] = msg.ecefVY*1e-2;
    ecef_ptr_->velocity[2] = msg.ecefVZ*1e-2;

    if (*ecef_pos_tow_ptr_ == *pvt_tow_ptr_ && 
        *ecef_pos_tow_ptr_ == *ecef_vel_tow_ptr_)
    {
        ecef_pub_->publish(*ecef_ptr_);
    }
}

void UBLOX_ROS::obsCB(const ublox::UBX_message_t &ubx_msg, uint8_t f9pID)
{
    ublox::RXM_RAWX_t msg = ubx_msg.RXM_RAWX;
    ublox_read_2::msg::ObsVec out;
    UTCTime utc =UTCTime::fromGPS(msg.week, msg.rcvTow*1e3);
    out.header.stamp.sec = utc.sec;
    out.header.stamp.nanosec = utc.nsec;
    for (int i = 0; i < msg.numMeas; i++)
    {
        out.obs[i].sat = msg.meas[i].svId;
        out.obs[i].gnss_id = msg.meas[i].gnssId;
        out.obs[i].signal = ublox::sigId(msg.meas[i].gnssId, msg.meas[i].sigId);
        switch (out.obs[i].signal)
        {
        case ublox_read_2::msg::Observation::GPS_L1_CA:
        case ublox_read_2::msg::Observation::GALILEO_E1_B:
        case ublox_read_2::msg::Observation::GALILEO_E1_C:
        case ublox_read_2::msg::Observation::QZSS_L1_CA:
            out.obs[i].freq = Ephemeris::GPS_FREQL1;
            break;
        case ublox_read_2::msg::Observation::GPS_L2_CL:
        case ublox_read_2::msg::Observation::GPS_L2_CM:
            out.obs[i].freq = Ephemeris::GPS_FREQL2;
            break;
        case ublox_read_2::msg::Observation::GLONASS_L1:
            out.obs[i].freq = GlonassEphemeris::FREQ1_GLO + msg.meas[i].freqId * GlonassEphemeris::DFRQ1_GLO;
            break;
        case ublox_read_2::msg::Observation::GLONASS_L2:
            out.obs[i].freq = GlonassEphemeris::FREQ2_GLO + msg.meas[i].freqId * GlonassEphemeris::DFRQ2_GLO;
            break;
            // These may not be right
//        case ublox_read_2::msg::Observation::GALILEO_E5_BI:
//        case ublox_read_2::msg::Observation::GALILEO_E5_BQ:
//            out.obs[i].freq = Ephemeris::GALILEO_FREQL5b;
//            break;
//        case ublox_read_2::msg::Observation::BEIDOU_B1I_D1:
//        case ublox_read_2::msg::Observation::BEIDOU_B1I_D2:
//            out.obs[i].freq = Ephemeris::BEIDOU_FREQ_B1;
//            break;
//        case ublox_read_2::msg::Observation::BEIDOU_B2I_D1:
//        case ublox_read_2::msg::Observation::BEIDOU_B2I_D2:
//            out.obs[i].freq = Ephemeris::BEIDOU_FREQ_B2;
//            break;
        default:
            out.obs[i].freq = 0;
            break;
        }
        out.obs[i].cno = msg.meas[i].cno;
        out.obs[i].locktime = msg.meas[i].locktime;
        out.obs[i].p = msg.meas[i].prMeas;
        out.obs[i].l = msg.meas[i].cpMeas;
        out.obs[i].d = msg.meas[i].doMeas;
        out.obs[i].stdevp = 0.01 * pow(2, msg.meas[i].prStdev);
        out.obs[i].stdevl = 0.004 * msg.meas[i].cpStdev;
        out.obs[i].stdevd = 0.002 * pow(2, msg.meas[i].doStdev);

        // indicate cycle slip
        if (msg.meas[i].cpMeas != 0.0
            && (msg.meas[i].trkStat & ublox::RXM_RAWX_t::trkStat_HalfCyc | ublox::RXM_RAWX_t::trkStat_subHalfCyc))
        {
            out.obs[i].lli =  ublox_read_2::msg::Observation::LLI_HALF_CYCLE_AMB;
        }
        else
        {
            out.obs[i].lli = 0;
        }
    }
    obs_pub_->publish(out);
}

void UBLOX_ROS::ephCB(const Ephemeris &eph)
{
    ublox_read_2::msg::Ephemeris out;
    out.header.stamp = this->now();

    out.sat = eph.sat;
    out.gnss_id = eph.gnssID;
    out.toe.sec = eph.toe.sec;
    out.toe.nanosec = eph.toe.nsec;
    out.toc.sec = eph.toc.sec;
    std::cerr<<"About to spin\n";
    out.toc.nanosec = eph.toc.nsec;

    out.tow = eph.tow;
    out.iodc = eph.iodc;
    out.iode = eph.iode;
    out.week = eph.week;
    out.toes = eph.toes;
    out.tocs = eph.tocs;
    out.health = eph.health;
    out.alert_flag = eph.alert_flag;
    out.anti_spoof = eph.anti_spoof;
    out.code_on_l2 = eph.code_on_L2;
    out.ura = eph.ura;
    out.l2_p_data_flag = eph.L2_P_data_flag;
    out.fit_interval_flag = eph.fit_interval_flag;
    out.age_of_data_offset = eph.age_of_data_offset;
    out.tgd[0] = eph.tgd[0];
    out.tgd[1] = eph.tgd[1];
    out.tgd[2] = eph.tgd[2];
    out.tgd[3] = eph.tgd[3];
    out.af2 = eph.af2;
    out.af1 = eph.af1;
    out.af0 = eph.af0;
    out.m0 = eph.m0;
    out.delta_n = eph.delta_n;
    out.ecc = eph.ecc;
    out.sqrta = eph.sqrta;
    out.omega0 = eph.omega0;
    out.i0 = eph.i0;
    out.w = eph.w;
    out.omegadot = eph.omegadot;
    out.idot = eph.idot;
    out.cuc = eph.cuc;
    out.cus = eph.cus;
    out.crc = eph.crc;
    out.crs = eph.crs;
    out.cic = eph.cic;
    out.cis = eph.cis;

    eph_pub_->publish(out);
}

void UBLOX_ROS::gephCB(const GlonassEphemeris &eph)
{
    ublox_read_2::msg::GlonassEphemeris out;
    out.header.stamp = this->now();

    out.sat = eph.sat;
    out.gnss_id = eph.gnssID;

    out.toe.sec = eph.toe.sec;
    out.toe.nanosec = eph.toe.nsec;
    out.tof.sec = eph.tof.sec;
    out.tof.nanosec = eph.tof.nsec;

    out.iode = eph.iode;
    out.frq = eph.frq;
    out.svh = eph.svh;
    out.sva = eph.sva;
    out.age = eph.age;
    out.pos[0] = eph.pos[0];
    out.pos[1] = eph.pos[1];
    out.pos[2] = eph.pos[2];
    out.vel[0] = eph.vel[0];
    out.vel[1] = eph.vel[1];
    out.vel[2] = eph.vel[2];
    out.acc[0] = eph.acc[0];
    out.acc[1] = eph.acc[1];
    out.acc[2] = eph.acc[2];
    out.taun = eph.taun;
    out.gamn = eph.gamn;
    out.dtaun = eph.dtaun;

    geph_pub_->publish(out);
}

void UBLOX_ROS::rtcmInputCB(const ublox::UBX_message_t &ubx_msg, uint8_t f9pID)
{
    ublox::RXM_RTCM_t msg = ubx_msg.RXM_RTCM;
    
    ublox_read_2::msg::RTCMInput out;
    out.header.stamp = this->now();
    out.version = msg.version;
    out.flags = msg.flags;
    out.crc_failed = msg.crcFailed;
    out.sub_type = msg.subType;
    out.ref_station = msg.refStation;
    out.msg_type = msg.msgType;

    rtcm_input_pub_->publish(out);
}

void UBLOX_ROS::rxmMeasxCB(const ublox::UBX_message_t &ubx_msg, uint8_t f9pID)
{
    ublox::RXM_MEASX_t msg = ubx_msg.RXM_MEASX;

    ublox_read_2::msg::SatelliteStatus out;
    out.version = msg.version;
    out.gps_tow = msg.gpsTOW;
    out.glo_tow = msg.gloTOW;
    out.bds_tow = msg.bdsTOW;
    out.qzss_tow = msg.qzssTOW;
    out.gps_tow_acc = msg.gpsTOWacc;
    out.glo_tow_acc = msg.gloTOWacc;
    out.bds_tow_acc = msg.bdsTOWacc;
    out.qzss_tow_acc = msg.qzssTOWacc;
    out.num_sv = msg.numSV;
    out.flags = msg.flags;

    for(uint8_t svIndex=0; svIndex<msg.numSV; svIndex++)
    {
        ublox_read_2::msg::Satellite sat;
        ublox::RXM_MEASX_t::SV_INFO_t msgsat = msg.sv[svIndex];
        sat.gnss_id = msgsat.gnssID;
        sat.sv_id = msgsat.svID;
        sat.carrier_noise_ratio = msgsat.cNo;
        sat.multipath_index = msgsat.mpathIndic;
        sat.doppler_meas = msgsat.dopplerMS;
        sat.dopper_hz = msgsat.dopplerHZ;
        sat.whole_chips = msgsat.wholeChips;
        sat.frac_chips = msgsat.fracChips;
        sat.code_phase = msgsat.codePhase;
        sat.int_code_phase = msgsat.intCodePhase;
        sat.pseu_range_rms_error = msgsat.pseuRangeRMSErr;
        out.satellites.push_back(sat);
    }

    sat_status_pub_->publish(out);
}
}