#ifndef UV_INDEX_H
#define UV_INDEX_H

#include <stdint.h>
#include "../mem_manager/mem_manager.h"
#include "../mem_manager/mem_map.h"

// Number of samples used in UV Index calibration
#define CALIBRATION_SAMPLES 3

namespace uv_sensor {

    void init();
    void tick();

    // Put the device in a known state
    void reset();

    // Starts the sampling state machine
    void take_reading();

    // Returns true if the device is currently running a sample batch
    bool is_running();

    // Returns data from LTR390 sensor
    void return_sensor_data();

    // Sets the UV sensitivty value in EEPROM
    void direct_calibrate(uint16_t value);

    // Calibrates the UV_sensor to an index
    // Takes a several samples and averages before calulating
    // the UV sensitivty
    void index_calibrate(float index);
}

#endif /* UV_INDEX_H */
