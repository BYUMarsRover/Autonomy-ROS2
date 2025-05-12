#include <stdint.h>
/*
 * AUTO-GENERATED FILE
 *
 * This file was automatically generated on 12 May 2025.
 * It directs to the appropriate function call using the command word and function address.
 *
 * DO NOT MODIFY THIS FILE DIRECTLY.
 * To update the contents of this file, please edit the function mapping CSV file
 * ('science_module_function_map.csv') and run the script 'build_switch_case.py'.
 *
 * Any manual changes to this file will be overwritten the next time the script is executed.
 */

#include <stdint.h>
#include "../error/error.h"
#include "science_module_functions.h"
#include "command_execution.h"

void executeQuery(uint8_t function_addr, uint8_t override) {
    switch (function_addr) {
        case 0: module_health_check(override); break;
        case 1: get_actuator_position(override); break;
        case 2: get_actuator_control(override); break;
        case 3: get_analog_sensor_raw(override); break;
        case 4: get_analog_sensor_calibrated(override); break;
        case 5: get_calibration_style(override); break;
        case 6: get_calibration_data(override); break;
        case 7: query_positional_controller(override); break;
        case 8: query_speed_controller(override); break;
        case 9: query_routine_controller(override); break;
        case 10: query_is_actuator_reserved(override); break;
        case 11: return_spectrograph_data(override); break;
        case 12: return_ltr_data(override); break;
        default:
            // No query configured
            error::badFunctionAddress(function_addr);
            break;
    }
}

void executeAction(uint8_t function_addr, uint8_t override) {
    switch (function_addr) {
        case 0: update_actuator_position(override); break;
        case 1: update_actuator_control(override); break;
        case 2: clear_analog_calibration(override); break;
        case 3: submit_analog_coefficients(override); break;
        case 4: submit_calibration_point(override); break;
        case 5: create_calibration_point(override); break;
        case 6: submit_positional_control(override); break;
        case 7: submit_speed_control(override); break;
        case 8: reserve_actuator(override); break;
        case 9: free_actuator(override); break;
        case 10: clear_positional_controller(override); break;
        case 11: clear_speed_controller(override); break;
        case 12: run_routine(override); break;
        case 13: run_routine_eeprom(override); break;
        case 14: pause_routine(override); break;
        case 15: resume_routine(override); break;
        case 16: step_routine(override); break;
        case 17: abort_routine(override); break;
        case 18: reset_spectrograph(override); break;
        case 19: sample_spectrograph(override); break;
        case 20: reset_ltr(override); break;
        case 21: sample_ltr(override); break;
        case 22: calibrate_uv_sensitivty(override); break;
        case 23: calibrate_uv_index(override); break;
        case 24: write_eeprom(override); break;
        default:
            // No command configured
            error::badFunctionAddress(function_addr);
            break;
    }
}

