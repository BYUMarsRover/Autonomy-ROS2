#ifndef MEM_MAP_H
#define MEM_MAP_H

/*
 * This file is linked to by the EEPROM config scripts.
 * Please update those references if you change the location of this file.
 * Please avoid sizeof() to allow for the config scripts to evaluate the #define statements
*/

#include <stdint.h>

#define EEPROM_SIZE 0x0400 // 1024 bytes
#define MAX_EEPROM_ADDR (EEPROM_SIZE - 1) // addr 0dx1023

// **Define Addresses for EEPROM variable here** //

// Config Data starts the write for the UV sensitivity value
// and the actuator extend/retract times here
#define EEPROM_CONFIG_START_ADDR 0x0000 

// UV Sensitivity Value
// uint16_t - 2 bytes
#define EEPROM_UV_SENSITIVTY_ADDR 0x0000

// Actuator Extend Retract Times
// 2 floats for each actuator - 8 bytes, 5 actuators - 40 bytes
#define EEPROM_ACTUATOR_EXTEND_RETRACT_TIME_TABLE_ADDR 0x0002

// Routine Lookup Table Size
// uint8_t - 1 byte
#define EEPROM_ROUTINE_LOOKUP_TABLE_SIZE_ADDR 0x002A

// Routine Lookup Table
// eeprom adresses uint16_t - 2 bytes each
#define EEPROM_ROUTINE_LOOKUP_TABLE_ADDR 0x002B

// This part of eeprom will hold custom routine data
// approx 0x002B - 0x03D7

// Only used when compiling for polynomial calibration
    // Information about the size of the coeff arrays
    #define NUM_COEFFS 10
    #define SIZE_OF_COEFF 4 // (sizeof(float))
    #define COEFF_ARRAY_SIZE (NUM_COEFFS * SIZE_OF_COEFF)

    // Put Humidity coefficients just before the Temperature coefficients
    #define EEPROM_PTR_HUM_COEFF_ARRAY (EEPROM_SIZE - COEFF_ARRAY_SIZE)
    #define EEPROM_PTR_HUM_COEFF_COUNT (EEPROM_PTR_HUM_COEFF_ARRAY - 1)

    // Put Temperature coefficients at the end of EEPROM
    #define EEPROM_PTR_TEMP_COEFF_ARRAY (EEPROM_PTR_HUM_COEFF_COUNT - COEFF_ARRAY_SIZE)
    #define EEPROM_PTR_TEMP_COEFF_COUNT (EEPROM_PTR_TEMP_COEFF_ARRAY - 1)


// Only used when compiling for linear interpolation
    // Information about the size of the calibration point arrays
    #define NUM_CALIBRATION_PTS 40
    #define SIZE_OF_CAL_PT 6 // (sizeof(calibration_point_t))
    #define CURVE_ARRAY_SIZE (NUM_CALIBRATION_PTS * SIZE_OF_CAL_PT)

    // Put Temperature coefficients at the end of EEPROM
    #define EEPROM_TEMP_CAL_PTS_COUNT (MAX_EEPROM_ADDR - COEFF_ARRAY_SIZE)
    #define EEPROM_TEMP_CAL_PTS_ARRAY (EEPROM_PTR_TEMP_COEFF_COUNT + 1)

    // Put Humidity points just before the Temperature points
    #define EEPROM_HUM_CAL_PTS_COUNT ((EEPROM_PTR_TEMP_COEFF_COUNT - 1) - COEFF_ARRAY_SIZE)
    #define EEPROM_HUM_CAL_PTS_ARRAY (EEPROM_PTR_HUM_COEFF_COUNT + 1)

#endif /* MEM_MAP_H */