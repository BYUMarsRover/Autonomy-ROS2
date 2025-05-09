/*
 * SCRIPT MAINTAINED FUNCTION DEFINITIONS
 *
 * Generated on: 05 April 2025
 *
 * This file is maintained by the 'build_function_implementations.py' script.
 * It ensures that all functions defined in the provided CSV mapping file
 * ('science_module_function_map.csv') are present in this file
 * and adds any missing function definitions.
 *
 * What you CAN modify:
 * - You may edit the function bodies to implement the desired functionality.
 * - You may add and remove #include statements and #define statements.
 *
 * What you CANNOT modify:
 * - Do not manually add or remove function definitions in this file.
 * - Do not modify the auto-generated include statements or function headers.
 *
 * Any manual changes to the auto-generated sections will be overwritten
 * the next time the script is executed.
 */

#include "science_module_functions.h"
#include "../definitions/definitions.h"
#include "../packet_comm/packet_comm.h"
#include "../error/error.h"
#include "../uv_sensor/uv_sensor.h"
#include "../spectrograph/spectrograph.h"
#include "../routine_manager/routine_manager.h"
#include "../actuator_manager/actuator_manager.h"
#include "../speed_controller/speed_controller.h"
#include "../positional_controller/positional_controller.h"
#include "../analog_curve/analog_curve.h"

// Returns a health check
void module_health_check(uint8_t override) {
    if (!verifyCommandOperandLength(0)) return;
    message::connection_successful();
}

// Returns the current position of actuator X
//    1: actuator_index (uint8_t) [0]
void get_actuator_position(uint8_t override) {
    if (!verifyCommandOperandLength(1)) return;
    uint8_t actuator_index = command_operand_buffer[0];
    uint8_t position = actuator_manager::get_position(actuator_index);
    writeToMessageBuffer(ERROR_CODE_SUCCESS, &position, 1);
}

// Returns the current signed control for actuator X
//    1: actuator_index (uint8_t) [0]
void get_actuator_control(uint8_t override) {
    if (!verifyCommandOperandLength(1)) return;
    uint8_t actuator_index = command_operand_buffer[0];

    // Extra case for drill
    if (actuator_index == 5) {
        writeToMessageBuffer(ERROR_CODE_SUCCESS, &DRILL_MOTOR_PINS.control, sizeof(DRILL_MOTOR_PINS.control));
        return;
    }

    uint8_t position = actuator_manager::get_control(actuator_index);
    writeToMessageBuffer(ERROR_CODE_SUCCESS, &position, 1);
}

// Returns the analog voltage returned by the Nano's 10 bit ADC for a given sensor index
//    1: sensor_index (uint8_t) [0]
void get_analog_sensor_raw(uint8_t override) {
    if (!verifyCommandOperandLength(1)) return;
    uint8_t sensor_index = command_operand_buffer[0];
    uint16_t raw = analogRead((sensor_index == 0) ? TEMPERATURE_PIN : MOISTURE_PIN);
    writeToMessageBuffer(ERROR_CODE_SUCCESS, &raw, sizeof(raw));
}

// Returns the calibrated value for a given sensor index
//    1: sensor_index (uint8_t) [0]
void get_analog_sensor_calibrated(uint8_t override) {
    if (!verifyCommandOperandLength(1)) return;
    uint8_t sensor_index = command_operand_buffer[0];
    float data = (sensor_index == 0) ? calcTemperatureCelsius() : calcRelativeHumidity();
    writeToMessageBuffer(ERROR_CODE_SUCCESS, &data, sizeof(data));
}

// Returns true if the nano is complied to run polynomial calibration, false if using 
// linear interpolation
void get_calibration_style(uint8_t override) {
    if (!verifyCommandOperandLength(0)) return;
    // Get Complied Curve Architecture
    #if SELECTED_CURVE_STYLE == POLY_COEFFS
        // Return 0 if using Polynomial Coefficients
        uint8_t response = 1;
    #else
        // Return 1 if using Linear Interpolation
        uint8_t response = 0;
    #endif
    writeToMessageBuffer(ERROR_CODE_SUCCESS, &response, sizeof(response));
}

// Returns a blob of data containing the calibration coefficients or points depending 
// on the compiled calibration style
//    1: sensor_index (uint8_t) [0]
void get_calibration_data(uint8_t override) {
    if (!verifyCommandOperandLength(1)) return;
    uint8_t sensor_index = command_operand_buffer[0];
    if (sensor_index == 0) echoTemperatureCurveData();
    else echoHumidityCurveData();
}

// Returns a string with information on the position controller
//    1: actuator_index (uint8_t) [0]
void query_positional_controller(uint8_t override) {
    if (!verifyCommandOperandLength(1)) return;
    uint8_t actuator_index = command_operand_buffer[0];
    positional_controller::report_to_message_buffer(actuator_index);
}

// Returns a string with information on the speed controller
//    1: actuator_index (uint8_t) [0]
void query_speed_controller(uint8_t override) {
    if (!verifyCommandOperandLength(1)) return;
    uint8_t actuator_index = command_operand_buffer[0];
    speed_controller::report_to_message_buffer(actuator_index);
}

// Returns a string with information on the routine controller
void query_routine_controller(uint8_t override) {
    if (!verifyCommandOperandLength(0)) return;
    routine_manager::report_to_message_buffer();
}

// Returns true if actuator X is reserved by a controller
//    1: actuator_index (uint8_t) [0]
void query_is_actuator_reserved(uint8_t override) {
    if (!verifyCommandOperandLength(1)) return;
    uint8_t actuator_index = command_operand_buffer[0];
    bool is_reserved = actuator_manager::is_reserved(actuator_index);
    writeToMessageBuffer(ERROR_CODE_SUCCESS, &is_reserved, sizeof(is_reserved));
}

// Returns 18 channel resulst cached in the spectrograph state machine
void return_spectrograph_data(uint8_t override) {
    if (!verifyCommandOperandLength(0)) return;

    // Ensure spectrograph is not allready running
    if (spectrograph::is_running()) {
        if (!override) {
            error::spectrographRunning();
            return;
        }
    }
    // Return spectrograph data
    spectrograph::return_spectrograph_data();
}

// Returns the calibrated UV Index and Lux conditions from the LTR390 sensor
void return_ltr_data(uint8_t override) {
    if (!verifyCommandOperandLength(0)) return;

    // Ensure UV sensor is not already running
    if (uv_sensor::is_running()) {
        if (!override) {
            error::uvSensorRunning();
            return;
        }
    }
    // Return uv sensor data
    uv_sensor::return_sensor_data();
}

// Updates the actuator position estimate to the provided value
//    1: actuator_index (uint8_t) [0]
//    2: new_position (uint8_t) [1]
void update_actuator_position(uint8_t override) {
    if (!verifyCommandOperandLength(2)) return;
    uint8_t actuator_index = command_operand_buffer[0];
    uint8_t new_position = command_operand_buffer[1];
    actuator_manager::set_position(actuator_index, new_position);
}

// Updates the applied actuator control to the provided value
//    1: actuator_index (uint8_t) [0]
//    2: new_control (int8_t) [1]
void update_actuator_control(uint8_t override) {
    if (!verifyCommandOperandLength(2)) return;
    uint8_t actuator_index = command_operand_buffer[0];
    int8_t new_control = command_operand_buffer[1];

    // Extra case for drill
    if (actuator_index == 5)
    {
        actuator_manager::set_drill_control(new_control);
        return;
    }

    // Ensure the actuator is not being controlled by a routine
    if (!override && !ensureActuatorIsFree(actuator_index)) {
        // ensureActuatorIsFree will send an error message, end the action
        return;
    }

    // Check for moving auger when the secondary is extended
    if (!override
        && actuator_index == AUGER_ACTUATOR_INDEX
        && actuator_state_array[SECONDARY_CACHE_ACTUATOR_INDEX].position > 0.01f) {
        // Secondary Cache is extended, and your trying to move the auger. Danger!

        error::augerLockout();

    // Check for moving secondary cahce when the auger is extended
    } else if (!override
        && actuator_index == SECONDARY_CACHE_ACTUATOR_INDEX
        && actuator_state_array[AUGER_ACTUATOR_INDEX].position > 0.01f) {
        // Auger is extended, and your trying to extend the secondary cache. Danger!

        error::secondCacheLockout();

    } else {
        actuator_manager::set_control(actuator_index, new_control);
    }
}

// Clears all calibration data for the provided analog sensor, function dependent on 
// calibration style
//    1: sensor_index (uint8_t) [0]
void clear_analog_calibration(uint8_t override) {
    if (!verifyCommandOperandLength(1)) return;

    uint8_t sensor_index = command_operand_buffer[0];
    if (sensor_index == 0) clearTemperatureCurveData();     
    else clearHumidityCurveData();
}

// Updates the analog sensor calibration with the provided coefficients, only works with 
// polynomial coefficient style
//    1: sensor_index (uint8_t) [0]
//    2: coefficients (float) [1-4]
void submit_analog_coefficients(uint8_t override) {
    if (!verifyValidCurveConfig(1)) return;
    uint8_t sensor_index = command_operand_buffer[0];
    if (sensor_index == 0) saveTemperatureCurveData(&(command_operand_buffer[1]), command_buffer.operand_byte_len);   
    else saveHumidityCurveData(&(command_operand_buffer[1]), command_buffer.operand_byte_len);
}

// Updates the analog sensor calibration with the provided points, only works with linear 
// interpolation style
//    1: sensor_index (uint8_t) [0]
//    2: points (calibration_point_t) [1-6]
void submit_calibration_point(uint8_t override) {
    #if SELECTED_CURVE_STYLE == LINEAR_INTERPOLATE
    submit_analog_coefficients(override);
    #else
    error::notCompiled();
    #endif
}

// Adds a calibration point to the provided sensor, only works with the linear interpolation 
// calibration style
//    1: sensor_index (uint8_t) [0]
//    2: external_value (float) [1-4]
void create_calibration_point(uint8_t override) {
    #if SELECTED_CURVE_STYLE == LINEAR_INTERPOLATE
    if (!verifyCommandOperandLength(5)) return;
    uint8_t sensor_index = command_operand_buffer[0];
    float external_value = *(float*)(&command_operand_buffer[1]);
    uint16_t raw = analogRead((sensor_index == 0) ? TEMPERATURE_PIN : MOISTURE_PIN);
    calibration_point_t pnt = { raw, external_value}

    if (sensor_index == 0) pushTempCalibrationPoint(pnt);
    else pushHumCalibrationPoint(pnt);
    #else
    error::notCompiled();
    #endif
}

// Reserves an actuator and drives it to the provided posiiton and the requested speed
//    1: actuator_index (uint8_t) [0]
//    2: position (uint8_t) [1]
//    3: speed (uint8_t) [2]
void submit_positional_control(uint8_t override) {
    if (!verifyCommandOperandLength(3)) return;
    uint8_t actuator_index = command_operand_buffer[0];
    uint8_t position = command_operand_buffer[1] / 255.0f;
    uint8_t speed = command_operand_buffer[2] / 255.0f;

    if (!override && !ensureActuatorIsFree(actuator_index)) return;
    positional_controller::submit(actuator_index, position, speed);
}

// Reserves and actuator and applies the provided control to for the requested time
//    1: actuator_index (uint8_t) [0]
//    2: control (int8_t) [1]
//    3: time_ms (uint32_t) [2-5]
void submit_speed_control(uint8_t override) {
    if (!verifyCommandOperandLength(6)) return;
    uint8_t actuator_index = command_operand_buffer[0];
    int8_t control = command_operand_buffer[1];
    uint32_t time = *((int32_t*)&(command_operand_buffer[2]));

    if (!override && !ensureActuatorIsFree(actuator_index)) return;

    if (!override 
        && actuator_index == SECONDARY_CACHE_ACTUATOR_INDEX
        && actuator_manager::get_position(AUGER_ACTUATOR_INDEX) > AUGER_SAFE_ZONE
        ) {
        error::secondCacheLockout();
        return;
    }

    if (!override 
        && actuator_index == AUGER_ACTUATOR_INDEX
        && actuator_manager::get_position(SECONDARY_CACHE_ACTUATOR_INDEX) > CACHE_SAFE_ZONE
        ) {
        error::augerLockout();
        return;
    }

    speed_controller::submit(actuator_index, control, time);
}

// Reserves the provided actuator
//    1: actuator_index (uint8_t) [0]
void reserve_actuator(uint8_t override) {
    if (!verifyCommandOperandLength(1)) return;
    uint8_t actuator_index = command_operand_buffer[0];

    // Ensure the actuator is not being controlled
    if (!override && actuator_manager::is_reserved(actuator_index)) {
        error::actuatorReserved(actuator_index);
        return;
    }
    actuator_manager::reserve(actuator_index);
}

// Frees the provided actuator
//    1: actuator_index (uint8_t) [0]
void free_actuator(uint8_t override) {
    if (!verifyCommandOperandLength(1)) return;
    uint8_t actuator_index = command_operand_buffer[0];

    // Ensure the actuator is not being controlled
    if (!override) {
        error::warnFreeActuator();
        return;
    }
    actuator_manager::free(actuator_index);
}

// Clears any pending actuator positional control
//    1: actuator_index (uint8_t) [0]
void clear_positional_controller(uint8_t override) {
    if (!verifyCommandOperandLength(1)) return;
    uint8_t actuator_index = command_operand_buffer[0];

    // Ensure no routine is running
    if (routine_manager::is_running()) {
        if (!override) {
            error::routineRunning();
            return;
        } else {
            routine_manager::abort();
        }
    }
    positional_controller::resolve(actuator_index);
}

// Clears any pending actuator speed control
//    1: actuator_index (uint8_t) [0]
void clear_speed_controller(uint8_t override) {
    if (!verifyCommandOperandLength(1)) return;
    uint8_t actuator_index = command_operand_buffer[0];

    // Check to make sure a routine is not running
    if (routine_manager::is_running()) {
        if (!override) {
            error::routineRunning();
        } else {
            routine_manager::abort();
        }
    }
    speed_controller::resolve(actuator_index);
}

// Begins the indicated routine
//    1: routine_index (uint8_t) [0]
void run_routine(uint8_t override) {
    if (!verifyCommandOperandLength(1)) return;
    uint8_t routine_index = command_operand_buffer[0];

    // Check to make sure a routine is not running
    if (routine_manager::is_running()) {
        if (override) {
            error::routineRunning();
            return;
        } else {
            routine_manager::abort();
        }
    }

    if (routine_index < ROUTINE_COUNT)
        routine_manager::begin_routine(ROUTINE_LOOKUP[routine_index]);
    else 
        error::badRoutineIndex(command_operand_buffer[0]);
}

// Pauses a running routine
void pause_routine(uint8_t override) {
    if (!verifyCommandOperandLength(0)) return;

    // Check to ensure a routine is running
    if (!routine_manager::is_running()) {
        error::noRoutineRunning();
    } else {
        if (routine_manager::is_paused()) error::routineAlreadyPaused();
        else routine_manager::pause();
    }
}

// Resumes a paused routine
void resume_routine(uint8_t override) {
    if (!verifyCommandOperandLength(0)) return;

    // Check to make ensure a routine is paused
    if (!routine_manager::is_paused()) error::noRoutinePaused();
    else routine_manager::resume();
}

// Moves a paused routine forward by one step
void step_routine(uint8_t override) {
    if (!verifyCommandOperandLength(0)) return;

    // Check to make sure a routine is running
    if (!routine_manager::is_paused()) error::noRoutinePaused();
    else routine_manager::step();
}

// Aborts a routine
void abort_routine(uint8_t override) {
    if (!verifyCommandOperandLength(0)) return;
    
    // Check to make sure a routine is running
    if (!routine_manager::is_running()) error::noRoutineRunning();
    else routine_manager::abort();
}

// Resets the spectrograph
void reset_spectrograph(uint8_t override) {
    if (!verifyCommandOperandLength(0)) return;
    spectrograph::reset();
}

// Samples the spectrograph
//    1: sample_cnt (uint8_t) [0]
//    2: sample_interval_ms (uint32_t) [1-4]
//    3: bulb_on (bool) [5]
void sample_spectrograph(uint8_t override) {
    if (!verifyCommandOperandLength(6)) return;
    uint8_t sample_cnt = command_operand_buffer[0];
    uint32_t sample_interval_mus = *((uint32_t*)&command_operand_buffer[1]);
    bool bulb_on = command_operand_buffer[5];

    // Check to make sure a sample is not being taken already
    if (spectrograph::is_running()) {
        if (!override) {
            error::spectrographRunning();
            return;
        }
    }

    // Begin a new sample run
    spectrograph::take_sample(sample_cnt, sample_interval_mus, bulb_on);
}

// No Docstring Provided
void reset_ltr(uint8_t override) {
    if (!verifyCommandOperandLength(0)) return;
    uv_sensor::reset();
}

// No Docstring Provided
void sample_ltr(uint8_t override) {
    if (!verifyCommandOperandLength(0)) return;
    // Ensure not already sampling
    if (uv_sensor::is_running()) {
        if (override) {
            error::uvSensorRunning();
            return;
        }
    }
    // Do the measurement
    uv_sensor::take_reading();
}

// No Docstring Provided
//    1: uv_sensitivty (uint16_t) [0-1]
void calibrate_uv_sensitivty(uint8_t override) {
    if (!verifyCommandOperandLength(2)) return;
    uint16_t uv_sensitivty = *((uint16_t*)command_operand_buffer);
    uv_sensor::direct_calibrate(uv_sensitivty);
}

// No Docstring Provided
//    1: uv_index (float) [0-3]
void calibrate_uv_index(uint8_t override) {
    if (!verifyCommandOperandLength(4)) return;
    float uv_index = *((float*)command_operand_buffer);
    uv_sensor::index_calibrate(uv_index);
}

