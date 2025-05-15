#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#include <stdint.h> // Uint8_t Datatype
#include <Arduino.h> // Analog Pin Aliases

/* DEBUG TOOLS */

// Uncomment this line to enable debug statements
//#define DEBUG

/* DEFINITIONS */

#define BAUD_RATE 115200
#define COMMAND_PACKET_HEADER 0x53 //'S'
#define COMMAND_PACKET_FOOTER 0x4D //'M'
#define RESPONSE_PACKET_HEADER 0x52 //'R'
#define RESPONSE_PACKET_FOOTER 0x46 //'F'

#define FULL_FORWARD_CONTROL 0x7F
#define FULL_REVERSE_CONTROL 0x80 //  0x81 for the drill instead , not sure why

// min time between switching motor directions
// (for shoot-through protection on H-bridges)
#define MIN_MOTOR_SWITCH_TIME_MS 100

/* PIN DEFINITIONS */ 

#define MOISTURE_PIN A1
#define TEMPERATURE_PIN A0

/* ARRAY SIZES */

#define LINEAR_ACTUATOR_CNT 5
#define TOOL_ACTUATOR_COUNT 2
#define CACHE_ACTUATOR_COUNT 3
#define MAX_OPERAND_ARRAY_SIZE 0xFF

#define DRILL_INDEX LINEAR_ACTUATOR_CNT
#define TOTAL_ACTUATOR_CNT LINEAR_ACTUATOR_CNT + 1 // Allow room for drill

/* HARDWARE CONSTANTS */

// Note: Feb 2025 -
// Discovered that the analog pins A6 and A7
// are analog input pins only and cannot output digital logics
// They are currently connected to the 2nd Motor Slot on Motor Driver Card 1
// Next iteration of PCB should use A6 and A7 for the Temp and Hum sensors
// and possibly route A2 and A3 to Motor Driver Card 1

#define PROBE_EXTEND_PIN    4
#define PROBE_RETRACT_PIN   5

#define AUGER_EXTEND_PIN    7
#define AUGER_RETRACT_PIN   6

#define PRIMARY_DOOR_EXTEND_PIN    12
#define PRIMARY_DOOR_RETRACT_PIN   10

#define SECONDARY_DOOR_EXTEND_PIN    8
#define SECONDARY_DOOR_RETRACT_PIN   9

#define SECONDARY_CACHE_EXTEND_PIN    2
#define SECONDARY_CACHE_RETRACT_PIN   3

/* OTHER CONSTANTS */

#define AUGER_SAFE_ZONE 0.03f
#define CACHE_SAFE_ZONE 0.03f

/* TYPES */

// Hardware structure for a linear actuator
struct linear_actuator_pins_t {
    uint8_t const extend;
    uint8_t const retract;
};

// Hardware structure for a linear actuator
// Loaded from EEPROM
struct linear_actuator_speed_t {
    float extend_rate_ms;
    float retract_rate_ms;
};

// State Estimation data for a linear actuator
struct linear_actuator_state_t {
    float position; // 0.0 Retracted -- 1.0 Extended
    int8_t control;  // -128 Retract -- 127 Extend
};

// pin def structure for a drill motor
struct drill_motor_pins_t {
    uint8_t enable;
    uint8_t direction; // (LOW: forward, HIGH: reverse)
    int8_t control;
};

// command packet structure
struct science_command {
    uint8_t read_header;
    uint8_t read_command_word;
    uint8_t operand_byte_len;
    uint8_t read_footer;
};

// response packet data
struct response_payload {
    uint8_t error_code;
    uint8_t message_byte_len;
};

extern drill_motor_pins_t DRILL_MOTOR_PINS;

/* MEMORY ALLOCATION */

#define PROBE_ACTUATOR_INDEX            0
#define AUGER_ACTUATOR_INDEX            1
#define PRIMARY_DOOR_ACTUATOR_INDEX     2
#define SECONDARY_DOOR_ACTUATOR_INDEX   3
#define SECONDARY_CACHE_ACTUATOR_INDEX  4
#define FIRST_TOOL_INDEX                PROBE_ACTUATOR_INDEX
#define FIRST_CACHE_INDEX               PRIMARY_DOOR_ACTUATOR_INDEX
extern linear_actuator_pins_t const ACTUATOR_PIN_ARRAY[];
extern linear_actuator_state_t actuator_state_array[LINEAR_ACTUATOR_CNT];

// Allocate memory space to handle commands
extern struct science_command command_buffer;
extern uint8_t command_operand_buffer[MAX_OPERAND_ARRAY_SIZE];

// Allocate memory space to handle responses
extern struct response_payload response_buffer;
extern uint8_t response_message_buffer[MAX_OPERAND_ARRAY_SIZE];

// Allocate memory space to hold actuator speed
extern struct linear_actuator_speed_t actuator_speed_array[LINEAR_ACTUATOR_CNT];

#endif /* DEFINITIONS_H */
