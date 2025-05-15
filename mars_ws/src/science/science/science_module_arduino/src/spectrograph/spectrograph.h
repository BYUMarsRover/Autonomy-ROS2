#ifndef SPECTROGRAPH_H
#define SPECTROGRAPH_H

#include <stdint.h>

namespace spectrograph {

    void init();
    void tick(unsigned long tick_period);

    // Put the device in a known state, reset all variables
    // Used if the data seems janky or nothing is happening
    // Trobleshooting tool
    void reset();

    // Starts the sampling statemachine
    // Should finish with a data packet being sent back to the rover
    void take_sample(uint8_t num_samples, uint32_t sample_interval_ms, bool with_bulb);

    // Returns true if the device is currently running a sample batch
    bool is_running();

    // Returns data from spectrograph
    void return_spectrograph_data();

}

#endif /* SPECTROGRAPH_H */
