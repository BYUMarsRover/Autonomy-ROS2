#include "command_execution.h"
#include "../definitions/definitions.h"
#include <Arduino.h>

void executeCommand() {
    // Check if the command is a command or a query
    bool isAction = (command_buffer.read_command_word & 0b10000000);
    uint8_t function_addr = (command_buffer.read_command_word & 0b00011111);
    bool overrideEnabled = (command_buffer.read_command_word & 0b01000000);

    // Branch depending on the action type
    if (isAction) executeAction(function_addr, overrideEnabled);
    else executeQuery(function_addr, overrideEnabled);
}

// void executeQuery(uint8_t function_addr) {
//     switch (function_addr) {
//         case 0b00000:
//         case 0b00001:
//         case 0b00010:
//         case 0b00011:
//         case 0b00100:
//         {
//             // Estimate Linear Actuator Position
//             uint8_t position = actuator_manager::get_position(function_address - 0b00000); // 0 - 5
//             writeToMessageBuffer(ERROR_CODE_SUCCESS, &position, 1);
//             break;
//         }
//         case 0b00101:
//         case 0b00110:
//         case 0b00111:
//         case 0b01000:
//         case 0b01001:
//         {
//             // Report Linear Actuator Control
//             int8_t control = actuator_manager::get_control(function_address - 0b00101); // 0 - 5
//             writeToMessageBuffer(ERROR_CODE_SUCCESS, &control, 1);
//             break;
//         }
//         case 0b01010:
//         {
//             // Report Drill Control
//             writeToMessageBuffer(ERROR_CODE_SUCCESS, &DRILL_MOTOR_PINS.control, sizeof(DRILL_MOTOR_PINS.control));
//             break;
//         }
//         case 0b01011:
//         {
//             // Echo Temp Coeffs or CSV Points
//             echoTemperatureCurveData();
//             break;
//         }
//         case 0b01100:
//         {
//             // Echo Hum Coeffs or CSV Points
//             echoHumidityCurveData();
//             break;
//         }
//         case 0b01101:
//         {
//             // Get Temperature Raw Analog
//             uint16_t temp_raw = analogRead(TEMPERATURE_PIN);
//             writeToMessageBuffer(ERROR_CODE_SUCCESS, &temp_raw, sizeof(temp_raw));
//             break;
//         }
        
//         case 0b01110:
//         {
//             // Get Humidity Raw Analog
//             uint16_t humidity_raw = analogRead(MOISTURE_PIN);
//             writeToMessageBuffer(ERROR_CODE_SUCCESS, &humidity_raw, sizeof(humidity_raw));
//             break;
//         }
//         case 0b01111:
//         {
//             // Get Temperature
//             float temp_C = calcTemperatureCelsius();
//             writeToMessageBuffer(ERROR_CODE_SUCCESS, &temp_C, sizeof(temp_C));
//             break;
//         }
//         case 0b10000:
//         {
//             // Get Humidity
//             float humidity_relative = calcRelativeHumidity();
//             writeToMessageBuffer(ERROR_CODE_SUCCESS, &humidity_relative, sizeof(humidity_relative));
//             break;
//         }
//         case 0b10001:
//         {
//             // Get Complied Curve Architecture
//             #if SELECTED_CURVE_STYLE == POLY_COEFFS
//                 // Return 0 if using Polynomial Coefficients
//                 uint8_t response = 0;
//             #else
//                 // Return 1 if using Linear Interpolation
//                 uint8_t response = 1;
//             #endif
//             writeToMessageBuffer(ERROR_CODE_SUCCESS, &response, sizeof(response));
//             break;
//         }
//         case 0b10010:
//         {
//             // Query Positional Controller
//             if (!verifyCommandOperandLength(1)) break;

//             // TODO add error checking on the index
//             positional_controller::report_to_message_buffer(command_operand_buffer[0]);
//             break;
//         }
//         case 0b10011:
//         {
//             // Query Speed Controller
//             if (!verifyCommandOperandLength(1)) break;

//             // TODO add error checking on the index
//             speed_controller::report_to_message_buffer(command_operand_buffer[0]);
//             break;
//         }
//         case 0b10100:
//         {
//             // Query Routine Status
//             if (!verifyCommandOperandLength(0)) break;

//             routine_manager::report_to_message_buffer();
//             break;
//         }
//         case 0b10101:
//         {
//             // Query Actuator Reserved Status
//             if (!verifyCommandOperandLength(1)) break;

//             bool is_reserved = actuator_manager::is_reserved(command_operand_buffer[0]);
//             writeToMessageBuffer(ERROR_CODE_SUCCESS, &is_reserved, sizeof(is_reserved));
//             break;
//         }
//         case 0b10110:
//         {
//             // Return Spectrograph Data
//             if (!verifyCommandOperandLength(0)) break;

//             if (spectrograph::is_running()) {
//                 if (!overrideEnabled)
//                 {
//                     error::spectrographRunning();
//                     break;
//                 }
//             }
//             // Return spectrograph data
//             spectrograph::return_spectrograph_data();
//             break;
//         }
//         case 0b10111:
//         {
//             // Return UV Sensor Data
//             if (!verifyCommandOperandLength(0)) break;

//             // Serial.println(uv_sensor::is_running());

//             if (uv_sensor::is_running()) {
//                 if (!overrideEnabled)
//                 {
//                     error::uvSensorRunning();
//                     break;
//                 }
//             }
//             // // Return uv sensor data
//             uv_sensor::return_sensor_data();
//             break;
//         }
//         default:
//         {
//             // No action configured
//             error::badFunctionAddress(function_address);
//             break;
//         }
//     }
// }

// void executeAction() {
//     uint8_t function_address = (command_buffer.read_command_word & 0b00011111);
//     bool overrideEnabled = (command_buffer.read_command_word & 0b01000000);

//     switch (function_address) {//still need to do error checking on size of operand. 
//         case 0b00000:
//         case 0b00001:
//         case 0b00010:
//         case 0b00011:
//         case 0b00100:
//         {
//             // Update linear actuator position
//             if (!verifyCommandOperandLength(1)) break;
//             actuator_manager::set_position(function_address - 0b00000, command_operand_buffer[0]);
//             break;
//         }
//         case 0b00101: 
//         case 0b00110:
//         case 0b00111:
//         case 0b01000:
//         case 0b01001:
//         {
//             // Update probe actuator control
//             if (!verifyCommandOperandLength(1)) break;
//             uint8_t actuator_index = function_address - 0b00101;

//             // Ensure the actuator is not being controlled by a routine
//             if (!overrideEnabled && !ensureActuatorIsFree(actuator_index)) {
//                 // ensureActuatorIsFree will send an error message, end the action
//                 break;
//             }

//             // Check for moving auger when the secondary is extended
//             if (!overrideEnabled
//                 && actuator_index == AUGER_ACTUATOR_INDEX
//                 && actuator_state_array[SECONDARY_CACHE_ACTUATOR_INDEX].position > 0.01f) {
//                 // Secondary Cache is extended, and your trying to move the auger. Danger!

//                 error::augerLockout();

//             // Check for moving secondary cahce when the auger is extended
//             } else if (!overrideEnabled
//                 && actuator_index == SECONDARY_CACHE_ACTUATOR_INDEX
//                 && actuator_state_array[AUGER_ACTUATOR_INDEX].position > 0.01f) {
//                 // Auger is extended, and your trying to extend the secondary cache. Danger!

//                 error::secondCacheLockout();

//             } else {
//                 actuator_manager::set_control(actuator_index, command_operand_buffer[0]);
//             }
//             break;
//         }
//         case 0b01010:
//         {
//             // Update drill control
//             if (!verifyCommandOperandLength(1)) break;
//             actuator_manager::set_drill_control(command_operand_buffer[0]);
//             break;
//         }
//         case 0b01011:
//         {
//             // Configure Temperature Curve
//             if (!verifyValidCurveConfig()) break;
//             saveTemperatureCurveData(command_operand_buffer, command_buffer.operand_byte_len);
//             break;
//         }
//         case 0b01100:
//         {
//             // Configure Humidity Curve
//             if (!verifyValidCurveConfig()) break;
//             saveHumidityCurveData(command_operand_buffer, command_buffer.operand_byte_len);
//             break;
//         }
//         case 0b01101:
//         {
//             // Clear Temperature Curve
//             if (!verifyCommandOperandLength(0)) break;
//             clearTemperatureCurveData();          
//             break;
//         }
//         case 0b01110:
//         {
//             // Clear Humidity Curve
//             if (!verifyCommandOperandLength(0)) break;
//             clearHumidityCurveData();
//             break;
//         }
//         case 0b01111:
//         {
//             // Positional Controller
//             if (!verifyCommandOperandLength(3)) break;

//             // Ensure the actuator is not being controlled
//             uint8_t actuator_index = command_operand_buffer[0];
//             if (!overrideEnabled && !ensureActuatorIsFree(actuator_index)) break;

//             float position = command_operand_buffer[1] / 255.0f;
//             float speed = command_operand_buffer[2] / 255.0f;
//             positional_controller::submit(actuator_index, position, speed);
//             break;
//         }
//         case 0b10000:
//         {
//             // Speed Controller
//             if (!verifyCommandOperandLength(6)) break;

//             // Ensure the actuator is not being controlled
//             uint8_t actuator_index = command_operand_buffer[0];
//             if (!overrideEnabled && !ensureActuatorIsFree(actuator_index)) break;

//             if (!overrideEnabled 
//                 && actuator_index == SECONDARY_CACHE_ACTUATOR_INDEX
//                 && actuator_manager::get_position(AUGER_ACTUATOR_INDEX) > AUGER_SAFE_ZONE
//                 ) {
//                 error::secondCacheLockout();
//                 break;
//             }

//             if (!overrideEnabled 
//                 && actuator_index == AUGER_ACTUATOR_INDEX
//                 && actuator_manager::get_position(SECONDARY_CACHE_ACTUATOR_INDEX) > CACHE_SAFE_ZONE
//                 ) {
//                 error::augerLockout();
//                 break;
//             }

//             uint8_t control = command_operand_buffer[1];
//             int32_t* timeout = (int32_t*)&(command_operand_buffer[2]);

//             #ifdef DEBUG
//             Serial.print("*** Timeout will be ");
//             Serial.print(*timeout);
//             Serial.println(" ***");
//             #endif

//             speed_controller::submit(actuator_index, control, *timeout);
//             break;
//         }
//         case 0b10001:
//         case 0b10010:
//         {
//             // Reserve Actuators
//             if (!verifyCommandOperandLength(1)) break;

//             uint8_t actuator_index = command_operand_buffer[0];

//             // Ensure the actuator is not being controlled
//             if (!overrideEnabled && actuator_manager::is_reserved(actuator_index)) {
//                 error::actuatorReserved(actuator_index);
//                 break;
//             }

//             actuator_manager::reserve(actuator_index);
//             break;
//         }
//         case 0b10011:
//         {
//             // Free Actuators
//             if (!verifyCommandOperandLength(1)) break;

//             // Ensure the actuator is not being controlled
//             if (!overrideEnabled) {
//                 error::warnFreeActuator();
//                 break;
//             }

//             uint8_t actuator_index = command_operand_buffer[0];
//             actuator_manager::free(actuator_index);
//             break;
//         }
//         case 0b10100:
//         {
//             // Clear Positional Controller
//             if (!verifyCommandOperandLength(1)) break;

//             // Check to make sure a routine is not running
//             if (routine_manager::is_running()) {
//                 if (!overrideEnabled) {
//                     error::routineRunning();
//                     break;
//                 } else {
//                     routine_manager::abort();
//                 }
//             }

//             uint8_t actuator_index = command_operand_buffer[0];
//             positional_controller::resolve(actuator_index);
//             break;
//         }
//         case 0b10101:
//         {
//             // Clear Speed Controller
//             if (!verifyCommandOperandLength(1)) break;

//             // Check to make sure a routine is not running
//             if (routine_manager::is_running()) {
//                 if (!overrideEnabled) {
//                     error::routineRunning();
//                     break;
//                 } else {
//                     routine_manager::abort();
//                 }
//             }

//             uint8_t actuator_index = command_operand_buffer[0];
//             speed_controller::resolve(actuator_index);
//             break;
//         }
//         case 0b10110:
//         {
//             // Pause/Abort Routine
//             if (!verifyCommandOperandLength(1)) break;

//             // Check to make sure a routine is running
//             if (!routine_manager::is_running()) {
//                 error::noRoutineRunning();
//                 break;
//             } else {
//                 if (command_operand_buffer[0] == 0) {
//                     routine_manager::abort();
//                 } else {
//                     if (routine_manager::is_paused()) {
//                         error::routineAlreadyPaused();
//                         break;
//                     } else {
//                         routine_manager::pause();
//                     }
//                 }
//             }

//             break;
//         }
//         case 0b10111:
//         {
//             // Resume/Step Routine
//             if (!verifyCommandOperandLength(1)) break;

//             // Check to make sure a routine is running
//             if (!routine_manager::is_paused()) {
//                 error::noRoutinePaused();
//                 break;
//             } else {
//                 if (command_operand_buffer[0] == 0)
//                     routine_manager::resume();
//                 else
//                     routine_manager::step();
//             }

//             break;
//         }
//         case 0b11000:
//         {
//             // Run a routine
//             if (!verifyCommandOperandLength(1)) break;

//             // Check to make sure a routine is not running
//             if (routine_manager::is_running()) {
//                 if (!overrideEnabled) {
//                     error::routineRunning();
//                     break;
//                 } else {
//                     routine_manager::abort();
//                 }
//             }

//             if (command_operand_buffer[0] < ROUTINE_COUNT)
//                 routine_manager::begin_routine(ROUTINE_LOOKUP[command_operand_buffer[0]]);
//             else 
//                 error::badRoutineIndex(command_operand_buffer[0]);
//             break;
//         }
//         case 0b11001:
//         {
//             // Instruct the spectrograph to begin taking samples
//             if (!verifyCommandOperandLength(6)) break;

//             // Check to make sure a sample is not being taken already
//             if (spectrograph::is_running()) {
//                 if (!overrideEnabled) {
//                     error::spectrographRunning();
//                     break;
//                 }
//             }

//             // Begin a new sample run
//             uint8_t num_samples = command_operand_buffer[0];
//             uint32_t sample_interval_mus = *((uint32_t*)&command_operand_buffer[1]);
//             bool bulb_state = command_operand_buffer[5];
//             spectrograph::take_sample(num_samples, sample_interval_mus, bulb_state);
//             break;
//         }
//         case 0b11010:
//         {
//             // Reset the spectrograph
//             if (!verifyCommandOperandLength(0)) break;
//             spectrograph::reset();
//             break;
//         }
//         case 0b11011:
//         {
//             // Take UV Sensor Data
//             if (!verifyCommandOperandLength(0)) break;

//             if (uv_sensor::is_running()) {
//                 if (!overrideEnabled)
//                 {
//                     error::uvSensorRunning();
//                     break;
//                 }
//             }
            
//             // Do the measurement
//             uv_sensor::take_reading();
//             break;
//         }
//         case 0b11100:
//         {
//             // Reset the uv_sensor
//             if (!verifyCommandOperandLength(0)) break;
//             uv_sensor::reset();
//             break;
//         }
//         case 0b11101:
//         {
//             // Calibrate the uv_sensor directly
//             if (!verifyCommandOperandLength(2)) break;
//             uv_sensor::direct_calibrate(*((uint16_t*)command_operand_buffer));
//             break;
//         }
//         case 0b11110:
//         {
//             // Calibrate the uv_sensor to an index
//             if (!verifyCommandOperandLength(4)) break;
//             uv_sensor::index_calibrate(*((float*)command_operand_buffer));
//             break;
//         }
//         default:
//         {
//             error::badFunctionAddress(function_address);
//             break;
//         }
//     }
// }
