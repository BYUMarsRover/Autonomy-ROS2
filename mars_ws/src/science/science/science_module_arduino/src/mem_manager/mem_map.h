#ifndef MEM_MAP_H
#define MEM_MAP_H

#include <stdint.h>

#define MAX_EEPROM_ADDR 0x03FF // addr 0dx1023

// **Define Addresses for EEPROM variable here** //

// UV Sensitivity Value
// uint16_t - 2 bytes
#define EEPROM_UV_SENSITIVTY_ADDR 0x0000
#define EEPROM_NEXT_ADDR EEPROM_UV_SENSITIVTY_ADDR + sizeof(uint16_t)

// Only used when compiling for polynomial calibration
    // Information about the size of the coeff arrays
    #define NUM_COEFFS 10
    #define SIZE_OF_COEFF (sizeof(float))
    #define COEFF_ARRAY_SIZE (NUM_COEFFS * SIZE_OF_COEFF)

    // Put Temperature coefficients at the end of EEPROM
    #define EEPROM_PTR_TEMP_COEFF_COUNT (MAX_EEPROM_ADDR - COEFF_ARRAY_SIZE)
    #define EEPROM_PTR_TEMP_COEFF_ARRAY (EEPROM_PTR_TEMP_COEFF_COUNT + 1)

    // Put Humidity coefficients just before the Temperature coefficients
    #define EEPROM_PTR_HUM_COEFF_COUNT (EEPROM_PTR_TEMP_COEFF_COUNT - COEFF_ARRAY_SIZE)
    #define EEPROM_PTR_HUM_COEFF_ARRAY (EEPROM_PTR_HUM_COEFF_COUNT + 1)


// Only used when compiling for linear interpolation
    // Information about the size of the calibration point arrays
    #define NUM_CALIBRATION_PTS 40
    #define SIZE_OF_CAL_PT (sizeof(calibration_point_t))
    #define CURVE_ARRAY_SIZE (NUM_CALIBRATION_PTS * SIZE_OF_CAL_PT)

    // Put Temperature coefficients at the end of EEPROM
    #define EEPROM_TEMP_CAL_PTS_COUNT (MAX_EEPROM_ADDR - COEFF_ARRAY_SIZE)
    #define EEPROM_TEMP_CAL_PTS_ARRAY (EEPROM_PTR_TEMP_COEFF_COUNT + 1)

    // Put Humidity points just before the Temperature points
    #define EEPROM_HUM_CAL_PTS_COUNT ((EEPROM_PTR_TEMP_COEFF_COUNT - 1) - COEFF_ARRAY_SIZE)
    #define EEPROM_HUM_CAL_PTS_ARRAY (EEPROM_PTR_HUM_COEFF_COUNT + 1)

#endif /* MEM_MAP_H */