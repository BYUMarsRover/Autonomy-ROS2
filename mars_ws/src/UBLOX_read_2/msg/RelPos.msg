std_msgs/Header header

uint16      ref_station_id      # Reference Station ID. Must be in the range 0..4095
float64[3]  rel_pos_ned         # NED component of relative position vector (m)
float64     rel_pos_length      # Length of relative position vector (m)
float64     rel_pos_heading     # Heading of the relative position vector. (rad)
float64[3]  rel_pos_hp_ned      # High precision NED, the measurment portion less than a mm. (m)
float64     rel_pos_hp_length   # High precision Length, the measurment portion less than a mm. (m)
float64[3]  acc_ned             # Accuracy of relative position North component (m)
float64     acc_length          # Accuracy of Length of the relative position vector (m)
float64     acc_heading         # Accuracy of heading of the relative position vector (rad)
uint32      flags               # See enums below
float64[3]  arrow_ned           # Difference vector from one rover to the other. (m)
float64     arrow_length        # Length of difference vector. (m)
float64[3]  arrow_rpy           # Roll/Pitch/Yaw from rover1 to rover2 (rad)

uint32 FLAGS_GNSS_FIX_OK            = 1
uint32 FLAGS_DIFF_SOLN              = 2
uint32 FLAGS_REL_POS_VALID          = 4
uint32 FLAGS_CARR_SOLN_FLOAT        = 8
uint32 FLAGS_CARR_SOLN_FIXED        = 16
uint32 FLAGS_IS_MOVING              = 32
uint32 FLAGS_REF_POS_MISS           = 64
uint32 FLAGS_REF_OBS_MISS           = 128
uint32 FLAGS_REL_POS_HEADING_VALID  = 256
