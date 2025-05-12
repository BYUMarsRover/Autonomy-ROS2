/*
Mars Rover 2024 - 2025 Development
File contains all constants, defintions, and datatypes
*/

#include "definitions.h"

/* MEMORY ALLOCATION */
const linear_actuator_pins_t ACTUATOR_PIN_ARRAY[] = {
    [PROBE_ACTUATOR_INDEX] = {
        // Probe Actuator
        .extend =           PROBE_EXTEND_PIN,
        .retract =          PROBE_RETRACT_PIN,
        .extend_rate =      PROBE_EXTEND_TIME,
        .retract_rate =     PROBE_RETRACT_TIME
    },
    [AUGER_ACTUATOR_INDEX] = {
        // Auger Actuator
        .extend =           AUGER_EXTEND_PIN,
        .retract =          AUGER_RETRACT_PIN,
        .extend_rate =      AUGER_EXTEND_TIME,
        .retract_rate =     AUGER_RETRACT_TIME
    },
    [PRIMARY_DOOR_ACTUATOR_INDEX] = {
        // Primary Door Actuator
        .extend =           PRIMARY_DOOR_EXTEND_PIN,
        .retract =          PRIMARY_DOOR_RETRACT_PIN,
        .extend_rate =      PRIMARY_DOOR_EXTEND_TIME,
        .retract_rate =     PRIMARY_DOOR_RETRACT_TIME
    },
    [SECONDARY_DOOR_ACTUATOR_INDEX] = {
        // Secondary Door Actuator
        .extend =           SECONDARY_DOOR_EXTEND_PIN,
        .retract =          SECONDARY_DOOR_RETRACT_PIN,
        .extend_rate =      SECONDARY_DOOR_EXTEND_TIME,
        .retract_rate =     SECONDARY_DOOR_RETRACT_TIME
    },
    [SECONDARY_CACHE_ACTUATOR_INDEX] = {
        // Secondary Cache Actuator
        .extend =           SECONDARY_CACHE_EXTEND_PIN,
        .retract =          SECONDARY_CACHE_RETRACT_PIN,
        .extend_rate =      SECONDARY_CACHE_EXTEND_TIME,
        .retract_rate =     SECONDARY_CACHE_RETRACT_TIME
    }
};

// Allocate memory space for state estimation
linear_actuator_state_t actuator_state_array[LINEAR_ACTUATOR_CNT];

drill_motor_pins_t DRILL_MOTOR_PINS = {
    .enable = 11,
    .direction = 13,
    .control = 0
};

// Allocate memory space to handle commands
struct science_command command_buffer;
uint8_t command_operand_buffer[MAX_OPERAND_ARRAY_SIZE];

// Allocate memory space to handle responses
struct response_payload response_buffer;
uint8_t response_message_buffer[MAX_OPERAND_ARRAY_SIZE];