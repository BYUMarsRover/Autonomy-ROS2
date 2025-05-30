#include "spectrograph.h"
#include "../error/error.h"
#include "../packet_comm/packet_comm.h"

#include <SparkFun_AS7265X.h>
#include <Wire.h>
#include <stdint.h>

// comment this line out to use calibrated values
#define RAW_VALUES
#define SPECTROGRAPH_NUM_CHANNELS 18

namespace spectrograph {

    // Hardware
    AS7265X sensor;
    float running_averages[SPECTROGRAPH_NUM_CHANNELS];
    bool use_bulb;

    // Configuration
    float sample_count;
    uint32_t sample_interval_mus;

    // State information
    int samples_taken;
    uint32_t timer;
    enum spectrograph_state_t {
        IDLE,
        START,
        DELAY,
        GRAPH,
    } spectrograph_state;

    void bulb_off() {
        // Turn spectrograph bulb off
        sensor.disableBulb(AS7265x_LED_WHITE); //Turn off bulb to avoid heating sensor
        sensor.disableBulb(AS7265x_LED_IR);
        sensor.disableBulb(AS7265x_LED_UV);
    }

    void bulb_on() {
        // Turn spectrograph bulb on
        sensor.enableBulb(AS7265x_LED_WHITE);
        sensor.enableBulb(AS7265x_LED_IR);
        sensor.enableBulb(AS7265x_LED_UV);
    }

    // Helper Function
    void start_sample_all_channels()
    {
        spectrograph_state = START;

        if (use_bulb) bulb_on();
        else bulb_off();

        // This will tell the module to measure all of the channels once
        sensor.setMeasurementMode(AS7265X_MEASUREMENT_MODE_6CHAN_ONE_SHOT);
    }

    void init() {
        reset();
    }

    void reset() {
        // Put the device in a known state, reset all variables
        // Used if the data seems janky or nothing is happening
        // Trobleshooting tool

        // Reset the running averages
        for (int i = 0; i < SPECTROGRAPH_NUM_CHANNELS; i++) {
            running_averages[i] = 0.0f;
        }

        // Go to default off state
        samples_taken = 0;
        spectrograph_state = IDLE;

        // Initialize the sensor
        if (!sensor.begin()) {
            error::spectrographNotFound();
        } else {
            message::spectrograph_found();
        }

        // Ensure bulb is off
        use_bulb = false;
        bulb_off();
    }

    void take_sample(uint8_t _sample_count, uint32_t sample_interval_ms, bool with_bulb) {
        // Starts the sampling statemachine
        // Should finish with a data packet being sent back to the rover
        if (sensor.begin() == false)
        {
            error::spectrographNotFound();
        }
        else {
            // Set the number of samples to take and the interval between them
            sample_count = _sample_count;
            sample_interval_mus = sample_interval_ms * 1e3;
            samples_taken = 0;

            // Clear out data from previous runs
            for (int i = 0; i < SPECTROGRAPH_NUM_CHANNELS; i++) {
                running_averages[i] = 0.0f;
            }

            // Set bulb state
            use_bulb = with_bulb;

            // Begin the sampling process
            start_sample_all_channels();
        } 
    }

    bool is_running() {
        return spectrograph_state != IDLE;
    }

    void tick(unsigned long tick_period) {
        switch (spectrograph_state) {
            case IDLE: {
                // Do nothing!
                break;
            }

            case START: {
                // Wait for the data to be available before reading the data
                if (!sensor.dataAvailable()) break;

                if (use_bulb) {  
                    running_averages[0] += sensor.getCalibratedA();   // 410nm
                    running_averages[1] += sensor.getCalibratedB();   // 435nm
                    running_averages[2] += sensor.getCalibratedC();   // 460nm
                    running_averages[3] += sensor.getCalibratedD();   // 485nm
                    running_averages[4] += sensor.getCalibratedE();   // 510nm
                    running_averages[5] += sensor.getCalibratedF();   // 535nm
                    running_averages[6] += sensor.getCalibratedG();   // 560nm
                    running_averages[7] += sensor.getCalibratedH();   // 585nm
                    running_averages[8] += sensor.getCalibratedR();   // 610nm
                    running_averages[9] += sensor.getCalibratedI();   // 645nm
                    running_averages[10] += sensor.getCalibratedS();   // 680nm
                    running_averages[11] += sensor.getCalibratedJ();   // 705nm
                    running_averages[12] += sensor.getCalibratedT();    // 730nm
                    running_averages[13] += sensor.getCalibratedU();   // 760nm
                    running_averages[14] += sensor.getCalibratedV();   // 810nm
                    running_averages[15] += sensor.getCalibratedW();   // 860nm
                    running_averages[16] += sensor.getCalibratedK();   // 900nm
                    running_averages[17] += sensor.getCalibratedL();   // 960nm    
                } else {
                    running_averages[0] += sensor.getA();   // 410nm
                    running_averages[1] += sensor.getB();   // 435nm
                    running_averages[2] += sensor.getC();   // 460nm
                    running_averages[3] += sensor.getD();   // 485nm
                    running_averages[4] += sensor.getE();   // 510nm
                    running_averages[5] += sensor.getF();   // 535nm
                    running_averages[6] += sensor.getG();   // 560nm
                    running_averages[7] += sensor.getH();   // 585nm
                    running_averages[8] += sensor.getR();   // 610nm
                    running_averages[9] += sensor.getI();   // 645nm
                    running_averages[10] += sensor.getS();   // 680nm
                    running_averages[11] += sensor.getJ();   // 705nm
                    running_averages[12] += sensor.getT();    // 730nm
                    running_averages[13] += sensor.getU();   // 760nm
                    running_averages[14] += sensor.getV();   // 810nm
                    running_averages[15] += sensor.getW();   // 860nm
                    running_averages[16] += sensor.getK();   // 900nm
                    running_averages[17] += sensor.getL();   // 960nm
                }

                samples_taken++;

                if (samples_taken < sample_count) {
                    timer = 0;
                    spectrograph_state = DELAY;
                } else {
                    spectrograph_state = GRAPH;
                }
                break;
            }

            case DELAY: {
                timer += tick_period;

                if (timer >= sample_interval_mus) {
                    start_sample_all_channels();
                }
                break;
            }
            case GRAPH: {

                // Divide the running averages by the number of samples taken
                for (int i = 0; i < SPECTROGRAPH_NUM_CHANNELS; i++) {
                    running_averages[i] /= sample_count; // Prints each element in the array
                }

                // Dont over heat sensor
                bulb_off();

                // Go back to idle state
                spectrograph_state = IDLE;
                break;
            }
        }
    }

    void return_spectrograph_data()
    {
        // Send the data back to the rover
        writeToMessageBuffer(ERROR_CODE_SUCCESS, running_averages, SPECTROGRAPH_NUM_CHANNELS * sizeof(float));
    }
}