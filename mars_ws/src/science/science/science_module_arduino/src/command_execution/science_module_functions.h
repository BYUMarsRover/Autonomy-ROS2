#ifndef SM_FUNCTIONS_H
#define SM_FUNCTIONS_H

/*
 * AUTO-GENERATED FILE
 *
 * This file was automatically generated on 12 May 2025.
 *
 * DO NOT MODIFY THIS FILE DIRECTLY.
 * To update the contents of this file, please edit the function mapping CSV file
 * ('science_module_function_map.csv') and run the script 'build_function_headers.py'.
 *
 * Any manual changes to this file will be overwritten the next time the script is executed.
 */

#include <stdint.h>

void module_health_check(uint8_t override); // Returns a health check
void get_actuator_position(uint8_t override); // Returns the current position of actuator X
void get_actuator_control(uint8_t override); // Returns the current signed control for actuator X
void get_analog_sensor_raw(uint8_t override); // Returns the analog voltage returned by the Nano's 10 bit ADC for a given sensor index
void get_analog_sensor_calibrated(uint8_t override); // Returns the calibrated value for a given sensor index
void get_calibration_style(uint8_t override); // Returns true if the nano is complied to run polynomial calibration, false if using linear interpolation
void get_calibration_data(uint8_t override); // Returns a blob of data containing the calibration coefficients or points depending on the compiled calibration style
void query_positional_controller(uint8_t override); // Returns a string with information on the position controller
void query_speed_controller(uint8_t override); // Returns a string with information on the speed controller
void query_routine_controller(uint8_t override); // Returns a string with information on the routine controller
void query_is_actuator_reserved(uint8_t override); // Returns true if actuator X is reserved by a controller
void return_spectrograph_data(uint8_t override); // Returns 18 channel resulst cached in the spectrograph state machine
void return_ltr_data(uint8_t override); // Returns the calibrated UV Index and Lux conditions from the LTR390 sensor
void update_actuator_position(uint8_t override); // Updates the actuator position estimate to the provided value
void update_actuator_control(uint8_t override); // Updates the applied actuator control to the provided value
void clear_analog_calibration(uint8_t override); // Clears all calibration data for the provided analog sensor, function dependent on calibration style
void submit_analog_coefficients(uint8_t override); // Updates the analog sensor calibration with the provided coefficients, only works with polynomial coefficient style
void submit_calibration_point(uint8_t override); // Updates the analog sensor calibration with the provided points, only works with linear interpolation style
void create_calibration_point(uint8_t override); // Adds a calibration point to the provided sensor, only works with the linear interpolation calibration style
void submit_positional_control(uint8_t override); // Reserves an actuator and drives it to the provided posiiton and the requested speed
void submit_speed_control(uint8_t override); // Reserves and actuator and applies the provided control to for the requested time
void reserve_actuator(uint8_t override); // Reserves the provided actuator
void free_actuator(uint8_t override); // Frees the provided actuator
void clear_positional_controller(uint8_t override); // Clears any pending actuator positional control
void clear_speed_controller(uint8_t override); // Clears any pending actuator speed control
void run_routine(uint8_t override); // Begins the indicated routine
void run_routine_eeprom(uint8_t override); // Begins the indicated routine from eeprom data
void pause_routine(uint8_t override); // Pauses a running routine
void resume_routine(uint8_t override); // Resumes a paused routine
void step_routine(uint8_t override); // Moves a paused routine forward by one step
void abort_routine(uint8_t override); // Aborts a routine
void reset_spectrograph(uint8_t override); // Resets the spectrograph
void sample_spectrograph(uint8_t override); // Samples the spectrograph
void reset_ltr(uint8_t override); // 
void sample_ltr(uint8_t override); // 
void calibrate_uv_sensitivty(uint8_t override); // 
void calibrate_uv_index(uint8_t override); // 
void write_eeprom(uint8_t override); // Writes data to the specified location in EEPROM

#endif /* SM_FUNCTIONS_H */
