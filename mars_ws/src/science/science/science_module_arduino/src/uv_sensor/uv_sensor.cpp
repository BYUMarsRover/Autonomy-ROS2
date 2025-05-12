#include "uv_sensor.h"
#include "../error/error.h"
#include "../packet_comm/packet_comm.h"
#include "../mem_manager/mem_manager.h"
#include "../mem_manager/mem_map.h"

#include <Wire.h>
#include <Adafruit_LTR390.h>
#include <stdint.h>

// See LTR390 datasheet
#define ALS_INT_TIME 1
#define ALS_GAIN 3
#define LUX_FACTOR (0.6 / (ALS_GAIN * ALS_INT_TIME))

// TODO set this up to live in EEPROM and be configurable
#define UV_SENSITIVITY 2300 // According to Datasheet

namespace uv_sensor {

    // Hardware
    Adafruit_LTR390 ltr = Adafruit_LTR390();

    // Cache
    float uv_data;  // UV Index
    float als_data; // Ambient Light

    // State information
    enum uv_sensor_state_t {
        IDLE,
        UV_CALIBRATION,
        UV,
        ALS
    } uv_sensor_state;

    uint8_t calibrating_cnt;
    float uv_calibration_index_cache;

    void enterUVmode()
    {
        // Serial.println(F("Configuring UV"));
        ltr.setMode(LTR390_MODE_UVS);
        ltr.setGain(LTR390_GAIN_18);                  //Recommended for UVI - x18
        ltr.setResolution(LTR390_RESOLUTION_20BIT);   //Recommended for UVI - 20-bit
        ltr.setThresholds(100, 1000);
        ltr.configInterrupt(true, LTR390_MODE_UVS);
    }

    void enterALSmode()
    {
        // Serial.println(F("Configuring ALS"));
        ltr.setMode(LTR390_MODE_ALS);
        ltr.setGain(LTR390_GAIN_3);                   //Recommended for Lux - x3
        ltr.setResolution(LTR390_RESOLUTION_18BIT);   //Recommended for Lux - 18-bit
        ltr.setThresholds(100, 1000);
        ltr.configInterrupt(true, LTR390_MODE_ALS);
    }

    float get_uvs_raw()
    {
        return float(ltr.readUVS());
    }

    float get_uv_index()
    {
        return get_uvs_raw() / EEPROM_readObject((uint16_t*)EEPROM_UV_SENSITIVTY_ADDR);
    }

    float get_lux()
    {
        return LUX_FACTOR * float(ltr.readALS());
    }

    void init() {
        reset();
    }

    void reset() {
        // Put the device in a known state

        // Go to default state
        uv_sensor_state = IDLE;
        uv_data = 0;
        als_data = 0;
        calibrating_cnt = 0;

        // Reset the sensor
        ltr.reset();

        // Initialize the sensor
        if (!ltr.begin()) {
            error::uvSensorNotFound();
        } else {
            message::uv_sensor_found();
        }
    }

    bool is_running() {
        return uv_sensor_state != IDLE;
    }

    void take_reading()
    {
        // Begin taking a reading from the sensor
        //Serial.println("Move to ALS State");
        // Serial.println(F("Taking Reading"));
        uv_sensor_state = UV;
        enterUVmode();
    }

    void tick() {
        switch (uv_sensor_state) {
            case IDLE: {
                // Do nothing!
                break;
            }

            case UV: {
                // Wait for the data to be available before reading the data
                if (!ltr.newDataAvailable()) break;

                // Read and use calibration setting
                uv_data = get_uv_index();

                uv_sensor_state = ALS;
                enterALSmode();
                break;
            }

            case ALS: {
                // Wait for the data to be available before reading the data
                if (!ltr.newDataAvailable()) break;
                als_data = get_lux();
                uv_sensor_state = IDLE;
                enterUVmode(); // This seems to be redundant, but it's required to make UV readings non zero *shrug emoji*
                break;
            }

            case UV_CALIBRATION: {

                // Perform a calibration sample
                if (!ltr.newDataAvailable()) break;
                uv_data += get_uvs_raw();
                enterUVmode();

                if ((--calibrating_cnt) == 0)
                {
                    // Finish calibration by averaging the results
                    uint16_t uv_sensitivty = uint16_t(uv_data / (CALIBRATION_SAMPLES * uv_calibration_index_cache));
                    direct_calibrate(uv_sensitivty);
                    uv_sensor_state = IDLE;
                }
            }

            default: {
                uv_sensor_state = IDLE;
                break;
            }
        }
    }

    void return_sensor_data()
    {
        // Send the data back to the controller
        float buffer[2];
        buffer[0] = uv_data;
        buffer[1] = als_data;
        writeToMessageBuffer(ERROR_CODE_SUCCESS, buffer, 2 * sizeof(float));
    }

    void index_calibrate(float index)
    {
        // Save and set up for calibration routine
        uv_calibration_index_cache = index;
        uv_data = 0;
        calibrating_cnt = max(CALIBRATION_SAMPLES, 0);

        // Start calibration
        uv_sensor_state = UV_CALIBRATION;
    }

    // Sets the UV sensitivty value in EEPROM
    void direct_calibrate(uint16_t value) {
        EEPROM_writeObject((uint16_t*)EEPROM_UV_SENSITIVTY_ADDR, value);
    }
}