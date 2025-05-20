#include "error.h"
#include "../definitions/definitions.h"
#include "../routine_manager/routine_manager.h"

// Helper Function
void composeErrorMessage(const char** error_format_strings, uint8_t error_string_cnt,
    uint16_t* args) {

    // Build a temp buffer to grab all the strings from memory
    uint8_t temp_buffer[MAX_OPERAND_ARRAY_SIZE];

    // Loop through the progmem error format strings
    uint8_t buffer_index = 0;
    for (uint8_t i = 0; i < error_string_cnt; i++) {
        // Add them to the temp buffer and inc the index by the len
        strcpy_P(&(temp_buffer[buffer_index]), error_format_strings[i]);
        buffer_index = buffer_index + strlen_P(error_format_strings[i]);
    }

    // Run a simple version of printf
    uint8_t* message_ptr = response_message_buffer;
    uint8_t* buffer_ptr = temp_buffer;
    uint8_t arg_index = 0;

    // Check if we are still in the reponse_message_buffer
    while (message_ptr - response_message_buffer < MAX_OPERAND_ARRAY_SIZE) {
        if (*buffer_ptr == '%') {
            // Copy the uint16_t value into the buffer
            message_ptr += snprintf(message_ptr, MAX_OPERAND_ARRAY_SIZE, "%d", args[arg_index++]);
            buffer_ptr++;
        } else {
            // Copy into the message buffer
            *(message_ptr++) = *(buffer_ptr++);
        }

        // Reached the end of the buffer_index
        if (buffer_ptr - temp_buffer == buffer_index) break;
    }

    // At this point a complete error message with filled in parameters should be sitting in
    // the error response buffer

    response_buffer.message_byte_len = message_ptr - response_message_buffer;
}

namespace message {

    void connection_successful() {
        const char* error_format_strings[] = {
            HEADER_MESS,
            MESS_SUCESSFUL_CONNECTION
        };
        response_buffer.error_code = ERROR_CODE_SUCCESS;
        composeErrorMessage(error_format_strings, sizeof(error_format_strings) / sizeof(const char*), nullptr);
    }

    void spectrograph_found() {
        const char* error_format_strings[] = {
            HEADER_MESS,
            NAME_SPECTROGRAPH,
            MESS_DEVICE_CONNECTED
        };
        response_buffer.error_code = ERROR_CODE_SUCCESS;
        composeErrorMessage(error_format_strings, sizeof(error_format_strings) / sizeof(const char*), nullptr);
    }

    void uv_sensor_found() {
        const char* error_format_strings[] = {
            HEADER_MESS,
            NAME_UV_SENSOR,
            MESS_DEVICE_CONNECTED
        };
        response_buffer.error_code = ERROR_CODE_SUCCESS;
        composeErrorMessage(error_format_strings, sizeof(error_format_strings) / sizeof(const char*), nullptr);
    }

    void uv_sensitivity_calibrate(uint16_t value) {
        const char* error_format_strings[] = {
            HEADER_MESS,
            MESS_UV_SENS_CALIBRATE
        };
        uint16_t args[] = {
            value
        };
        response_buffer.error_code = ERROR_CODE_SUCCESS;
        composeErrorMessage(error_format_strings, sizeof(error_format_strings) / sizeof(const char*), args);
    }
}

namespace error {

    void badOperandLengthFixed(uint8_t expected_length) {
        const char* error_format_strings[] = {
            HEADER_ERROR,
            MESS_RECEIVED_X_BYTES_OPERAND_PACKET,
            ERROR_BAD_OPERAND_PACKET_SIZE
        };
        uint16_t args[] = {
            command_buffer.operand_byte_len,
            expected_length
        };
        response_buffer.error_code = ERROR_CODE_GENERIC_ERROR;
        composeErrorMessage(error_format_strings, sizeof(error_format_strings) / sizeof(const char*), args);
    }

    void badOperandLengthModulus(uint8_t modulo) {
        const char* error_format_strings[] = {
            HEADER_ERROR,
            MESS_RECEIVED_X_BYTES_OPERAND_PACKET,
            ERROR_BAD_OPERAND_PACKET_VAR
        };
        uint16_t args[] = {
            command_buffer.operand_byte_len,
            modulo
        };
        response_buffer.error_code = ERROR_CODE_BAD_OPERANDS;
        composeErrorMessage(error_format_strings, sizeof(error_format_strings) / sizeof(const char*), args);
    }

    void actuatorReserved(uint8_t actuator_index) {
        const char* error_format_strings[] = {
            HEADER_ERROR,
            ERROR_ACTUATOR_RESERVED
        };
        uint16_t args[] = { actuator_index };
        response_buffer.error_code = ERROR_CODE_ACTUATOR_RESERVED;
        composeErrorMessage(error_format_strings, sizeof(error_format_strings) / sizeof(const char*), args);
    }

    void warnFreeActuator() {
        const char* error_format_strings[] = {
            HEADER_WARNING,
            WARN_FREE_ACTUATOR,
            WARN_ADVANCE_INSTRUCTIONS
        };
        response_buffer.error_code = ERROR_CODE_WARN;
        composeErrorMessage(error_format_strings, sizeof(error_format_strings) / sizeof(const char*), nullptr);
    }

    void badFunctionAddress(uint8_t addr) {
        const char* error_format_strings[] = {
            HEADER_ERROR,
            ERROR_UNRECOGNIZED_FUNCTION_ADDR
        };
        uint16_t args[] = { addr };
        response_buffer.error_code = ERROR_CODE_GENERIC_ERROR;
        composeErrorMessage(error_format_strings, sizeof(error_format_strings) / sizeof(const char*), args);
    }

    void routineRunning() {
        const char* error_format_strings[] = {
            HEADER_WARNING,
            WARN_ROUTINE_RUNNING,
            WARN_ADVANCE_INSTRUCTIONS
        };
        response_buffer.error_code = ERROR_CODE_DANGEROUS_ACTION;
        composeErrorMessage(error_format_strings, sizeof(error_format_strings) / sizeof(const char*), nullptr);
    }

    void noRoutinePaused() {
        const char* error_format_strings[] = {
            HEADER_ERROR,
            ERROR_NOTHING_PAUSED
        };
        response_buffer.error_code = ERROR_CODE_BAD_STATE;
        composeErrorMessage(error_format_strings, sizeof(error_format_strings) / sizeof(const char*), nullptr);
    }

    void noRoutineRunning() {
        const char* error_format_strings[] = {
            HEADER_ERROR,
            ERROR_NO_ROUTINE_RUNNING
        };
        response_buffer.error_code = ERROR_CODE_BAD_STATE;
        composeErrorMessage(error_format_strings, sizeof(error_format_strings) / sizeof(const char*), nullptr);
    }

    void routineAlreadyPaused() {
        const char* error_format_strings[] = {
            HEADER_ERROR,
            ERROR_ALREADY_PAUSED
        };
        response_buffer.error_code = ERROR_CODE_BAD_STATE;
        composeErrorMessage(error_format_strings, sizeof(error_format_strings) / sizeof(const char*), nullptr);
    }

    void augerLockout() {
        const char* error_format_strings[] = {
            HEADER_ERROR,
            ERROR_2ND_CACHE_EXTENED_CANNOT_MOVE_AUGER
        };
        response_buffer.error_code = ERROR_CODE_DANGEROUS_ACTION;
        composeErrorMessage(error_format_strings, sizeof(error_format_strings) / sizeof(const char*), nullptr);
    }

    void secondCacheLockout() {
        const char* error_format_strings[] = {
            HEADER_ERROR,
            ERROR_AUGER_EXTENED_CANNOT_MOVE_2ND_CACHE
        };
        response_buffer.error_code = ERROR_CODE_DANGEROUS_ACTION;
        composeErrorMessage(error_format_strings, sizeof(error_format_strings) / sizeof(const char*), nullptr);
    }

    void spectrographRunning() {
        const char* error_format_strings[] = {
            HEADER_ERROR,
            NAME_SPECTROGRAPH,
            ERROR_DEVICE_RUNNING
        };
        response_buffer.error_code = ERROR_CODE_BAD_STATE;
        composeErrorMessage(error_format_strings, sizeof(error_format_strings) / sizeof(const char*), nullptr);
    }

    void spectrographNotFound() {
        const char* error_format_strings[] = {
            HEADER_ERROR,
            NAME_SPECTROGRAPH,
            ERROR_DEVICE_NOT_FOUND
        };
        response_buffer.error_code = ERROR_CODE_BAD_STATE;
        composeErrorMessage(error_format_strings, sizeof(error_format_strings) / sizeof(const char*), nullptr);
    }

    void uvSensorRunning() {
        const char* error_format_strings[] = {
            HEADER_ERROR,
            NAME_UV_SENSOR,
            ERROR_DEVICE_RUNNING
        };
        response_buffer.error_code = ERROR_CODE_BAD_STATE;
        composeErrorMessage(error_format_strings, sizeof(error_format_strings) / sizeof(const char*), nullptr);
    }

    void uvSensorNotFound() {
        const char* error_format_strings[] = {
            HEADER_ERROR,
            NAME_UV_SENSOR,
            ERROR_DEVICE_NOT_FOUND
        };
        response_buffer.error_code = ERROR_CODE_BAD_STATE;
        composeErrorMessage(error_format_strings, sizeof(error_format_strings) / sizeof(const char*), nullptr);
    }

    void notImplemented() {
        const char* error_format_strings[] = {
            HEADER_ERROR,
            ERROR_NOT_IMPLEMENTED
        };
        response_buffer.error_code = ERROR_CODE_BAD_STATE;
        composeErrorMessage(error_format_strings, sizeof(error_format_strings) / sizeof(const char*), nullptr);
    }

    void notCompiled() {
        const char* error_format_strings[] = {
            HEADER_ERROR,
            ERROR_NOT_COMPILED
        };
        response_buffer.error_code = ERROR_CODE_GENERIC_ERROR;
        composeErrorMessage(error_format_strings, sizeof(error_format_strings) / sizeof(const char*), nullptr);
    }

    void nullptrError() {
        const char* error_format_strings[] = {
            HEADER_ERROR,
            ERROR_NULL_PTR
        };
        response_buffer.error_code = ERROR_CODE_GENERIC_ERROR;
        composeErrorMessage(error_format_strings, sizeof(error_format_strings) / sizeof(const char*), nullptr);
    }

    void mallocError() {
        const char* error_format_strings[] = {
            HEADER_ERROR,
            ERROR_MALLOC
        };
        response_buffer.error_code = ERROR_CODE_GENERIC_ERROR;
        composeErrorMessage(error_format_strings, sizeof(error_format_strings) / sizeof(const char*), nullptr);
    }

    void outOfBoundsError(uint8_t index, uint8_t array_size) {
        const char* error_format_strings[] = {
            HEADER_ERROR,
            ERROR_OUT_OF_BOUNDS
        };
        uint16_t args[] = {
            index,
            array_size
        };
        response_buffer.error_code = ERROR_CODE_GENERIC_ERROR;
        composeErrorMessage(error_format_strings, sizeof(error_format_strings) / sizeof(const char*), args);
    }

    void notReadyForRoutine(uint8_t index) {
        const char* error_format_strings[] = {
            HEADER_ERROR,
            ERROR_COULD_NOT_RUN_ROUTINE,
            ERROR_ACTUATOR_RESERVED
        };
        uint16_t args[] = {
            index
        };
        response_buffer.error_code = ERROR_CODE_ACTUATOR_RESERVED;
        composeErrorMessage(error_format_strings, sizeof(error_format_strings) / sizeof(const char*), args);
    }

    void notReadyForCacheAlignment() {
        const char* error_format_strings[] = {
            HEADER_ERROR,
            ERROR_COULD_NOT_RUN_ROUTINE,
            ERROR_AUGER_SECONDAY_NON_ZERO
        };
        response_buffer.error_code = ERROR_CODE_ACTUATOR_RESERVED;
        composeErrorMessage(error_format_strings, sizeof(error_format_strings) / sizeof(const char*), nullptr);
    }

    void badIndex(const char* name, uint8_t index, uint8_t max_index) {
        const char* error_format_strings[] = {
            HEADER_ERROR,
            name,
            ERROR_INDEX_NOT_RECOGNIZED,
            ERROR_OUT_OF_BOUNDS
        };
        uint16_t args[] = {
            index,
            index,
            max_index
        };
        response_buffer.error_code = ERROR_CODE_GENERIC_ERROR;
        composeErrorMessage(error_format_strings, sizeof(error_format_strings) / sizeof(const char*), args);
    }
}

namespace verify {

    bool verify(const char* name, uint8_t index, uint8_t max) {
        if (index >= max) {
            error::badIndex(name, index, max);
            return false;
        } else return true;
    }

    bool routine_index(uint8_t routine_index) {
        uint8_t total_routine_count = routine_manager::get_total_routine_count();
        return verify(NAME_ROUTINE, routine_index, total_routine_count);
    }

    bool actuator_index(uint8_t actuator_index) {
        return verify(NAME_ACTUATOR, actuator_index, TOTAL_ACTUATOR_CNT);
    }

    bool linear_actuator_index(uint8_t linear_actuator_index) {
        return verify(NAME_ACTUATOR, linear_actuator_index, LINEAR_ACTUATOR_CNT);
    }

}