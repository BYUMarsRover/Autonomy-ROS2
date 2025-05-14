#ifndef ERROR_H
#define ERROR_H

#include <Arduino.h>

/* ERROR CODES */

#define ERROR_CODE_SUCCESS 0
#define ERROR_CODE_WARN 1
#define ERROR_CODE_GENERIC_ERROR 2
#define ERROR_CODE_CMD_UNRECOGNIZED 2
#define ERROR_CODE_BAD_OPERANDS 3
#define ERROR_CODE_DANGEROUS_ACTION 4
#define ERROR_CODE_ACTUATOR_RESERVED 5
#define ERROR_CODE_BAD_STATE 6

const char HEADER_ERROR[] PROGMEM = "ERR: ";
const char HEADER_WARNING[] PROGMEM = "WARN: ";
const char HEADER_MESS[] PROGMEM = "MESS: ";
const char NAME_SPECTROGRAPH[] PROGMEM = "Spectral Traid ";
const char NAME_UV_SENSOR[] PROGMEM = "LTR390 ";
const char NAME_ROUTINE[] PROGMEM = "Routine ";
const char NAME_ACTUATOR[] PROGMEM = "Actuator ";
const char MESS_SUCESSFUL_CONNECTION[] PROGMEM = "Connection Sucessful";
const char MESS_DEVICE_CONNECTED[] PROGMEM = "found and connected.";
const char ERROR_2ND_CACHE_EXTENED_CANNOT_MOVE_AUGER[] PROGMEM = "Secondary Cache is Extended. Cannot extend Auger.";
const char ERROR_AUGER_EXTENED_CANNOT_MOVE_2ND_CACHE[] PROGMEM = "Auger is Extended. Cannot extend secondary cache.";
const char WARN_FREE_ACTUATOR[] PROGMEM = "Freeing a Actuator could cause program conflicts.";
const char WARN_ROUTINE_RUNNING[] PROGMEM = "A routine is currently running.";
const char WARN_DISABLE_POS_CONTROLLER[] PROGMEM = "Disabling the positional controller will abort the routine.";
const char WARN_DISABLE_SPEED_CONTROLLER[] PROGMEM = "Disabling the speed controller will abort the routine.";
const char ERROR_NO_ROUTINE_RUNNING[] PROGMEM = "No routine is running.";
const char ERROR_ALREADY_PAUSED[] PROGMEM = "Routine is already paused.";
const char ERROR_NOTHING_PAUSED[] PROGMEM = "No paused routine to resume.";
const char WARN_ADVANCE_INSTRUCTIONS[] PROGMEM = "Set the override bit if you wish to continue.";
const char ERROR_INDEX_NOT_RECOGNIZED[] PROGMEM = "Index % was not recognized.";
const char MESS_RECEIVED_X_BYTES_OPERAND_PACKET[] PROGMEM = "Received % bytes in operand packet ";
const char ERROR_BAD_OPERAND_PACKET_SIZE[] PROGMEM = "but expected %.";
const char ERROR_BAD_OPERAND_PACKET_VAR[] PROGMEM = "but expected n * %.";
const char ERROR_ACTUATOR_RESERVED[] PROGMEM = "Actuator[%] control is reserved.";
const char ERROR_UNRECOGNIZED_FUNCTION_ADDR[] PROGMEM = "Unrecognized Function Address: %";
const char ERROR_DEVICE_RUNNING[] PROGMEM = "is already sampling.";
const char ERROR_DEVICE_NOT_FOUND[] PROGMEM = "is not connected, check wiring.";
const char ERROR_NOT_IMPLEMENTED[] PROGMEM = "This function is not yet implemented.";
const char ERROR_NOT_COMPILED[] PROGMEM = "This function is included in this build.";
const char ERROR_NULL_PTR[] PROGMEM = "A nullptr was encountered.";
const char ERROR_MALLOC[] PROGMEM = "Memory was unable to be allocated.";
const char ERROR_OUT_OF_BOUNDS[] PROGMEM = "Out of bounds access, received index % in an array of length %.";
const char ERROR_COULD_NOT_RUN_ROUTINE[] PROGMEM = "Could not run routine.";

namespace message {
    void connection_successful();
    void spectrograph_found();
    void uv_sensor_found();
}

namespace error {
    void badOperandLengthFixed(uint8_t expected_length);
    void badOperandLengthModulus(uint8_t modulo);
    void actuatorReserved(uint8_t actuator_index);
    void warnFreeActuator();
    void badFunctionAddress(uint8_t addr);
    void routineRunning();
    void noRoutinePaused();
    void noRoutineRunning();
    void routineAlreadyPaused();
    void augerLockout();
    void secondCacheLockout();
    void spectrographRunning();
    void spectrographNotFound();
    void uvSensorRunning();
    void uvSensorNotFound();
    void notImplemented();
    void notCompiled();
    void nullptrError();
    void mallocError();
    void outOfBoundsError(uint8_t index, uint8_t array_size);
    void badIndex(const char* name, uint8_t index, uint8_t max_index);
    void notReadyForRoutine(uint8_t index);
}

namespace verify {
    bool routine_index(uint8_t index);
    bool actuator_index(uint8_t index);
    bool linear_actuator_index(uint8_t index);
}

#endif /* ERROR_H */
