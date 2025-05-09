#include "hardware.h"
#include "../definitions/definitions.h"
#include <stdint.h>

void setPinModes() {
    // Loop over all linear actuators
    for (uint8_t i = 0; i < LINEAR_ACTUATOR_CNT; i++) {
        pinMode(ACTUATOR_PIN_ARRAY[i].extend, OUTPUT);
        pinMode(ACTUATOR_PIN_ARRAY[i].retract, OUTPUT);
    }

    // soil sensor probe
    pinMode(MOISTURE_PIN, INPUT);
    pinMode(TEMPERATURE_PIN, INPUT);
    //analogReference(EXTERNAL);

    // drill motor
    pinMode(DRILL_MOTOR_PINS.enable, OUTPUT);
    pinMode(DRILL_MOTOR_PINS.direction, OUTPUT);
}

// return whether the given pin is currently under digitalWrite(pin, HIGH)
bool isDigitalWrittenHigh(uint8_t pin) {
  return (0 != (*portOutputRegister(digitalPinToPort(pin)) &
                digitalPinToBitMask(pin)));
}

// halt a linear actuator's movement
void stopActuator(uint8_t actuator_index) {
    // Stop extending and retracting
    linear_actuator_pins_t pins = ACTUATOR_PIN_ARRAY[actuator_index];
    digitalWrite(pins.extend, LOW);
    digitalWrite(pins.retract, LOW);
    actuator_state_array[actuator_index].control = 0;
}

// halt a tool's actuators
void stopToolActuator(uint8_t tool_index) {
    stopActuator(FIRST_TOOL_INDEX + tool_index);
    // Stop the drill if its required
    if (tool_index == AUGER_ACTUATOR_INDEX) {
        digitalWrite(DRILL_MOTOR_PINS.enable, LOW);
    }
}

// stop all tool actuators
void stopAllToolActuators() {
    for (uint8_t i = 0; i < TOOL_ACTUATOR_COUNT; i++) {
        stopToolActuator(i);
    }
}

// halt a cache's actuators
void stopCacheActuator(uint8_t cache_index) {
    stopActuator(FIRST_CACHE_INDEX + cache_index);
}

// stop all cache actuators
void stopAllCacheActuators() {
    for (uint8_t i = 0; i < CACHE_ACTUATOR_COUNT; i++) {
        stopCacheActuator(i);
    }
}

void stopAllActuators() {
    stopAllToolActuators();
    stopAllCacheActuators();
}

void writeActuator(uint8_t actuator_index, int8_t speed) {

    linear_actuator_pins_t pins = ACTUATOR_PIN_ARRAY[actuator_index];
    actuator_state_array[actuator_index].control = speed;

    if (speed == 0) stopActuator(actuator_index);
    else if (speed < 0) {
        
        // Halt Retracting
        if (isDigitalWrittenHigh(pins.retract)) {
            // shoot-through protection
            digitalWrite(pins.retract, LOW);
            delay(MIN_MOTOR_SWITCH_TIME_MS);
        } else digitalWrite(pins.retract, LOW);

        // Begin Extending
        if (actuator_index >= FIRST_CACHE_INDEX) {
            // This is a cache actuator
            digitalWrite(pins.extend, HIGH);
        }
        else {
            // This is a tool actuator
            analogWrite(pins.extend, map(speed, 0, -128, 0, 255));
        }
        
    }
    else {

        // Halt Extending
        if (isDigitalWrittenHigh(pins.extend)) {
            // shoot-through protection
            digitalWrite(pins.extend, LOW);
            delay(MIN_MOTOR_SWITCH_TIME_MS);
        } else digitalWrite(pins.extend, LOW);

        // Begin Retracting
        if (actuator_index >= FIRST_CACHE_INDEX) {
            // This is a cache actuator
            digitalWrite(pins.retract, HIGH);
        }
        else {
            // This is a tool actuator
            analogWrite(pins.retract, map(speed, 0, 127, 0, 255));
        }
    }
}

void stopDrill() {
    digitalWrite(DRILL_MOTOR_PINS.enable, LOW);
}

void writeDrill(int8_t drill_speed) { 
    DRILL_MOTOR_PINS.control = command_operand_buffer[0];
    if (drill_speed > 0) {
        // drill forward
        if (isDigitalWrittenHigh(DRILL_MOTOR_PINS.direction)) {
            // shoot-through protection
            digitalWrite(DRILL_MOTOR_PINS.enable, LOW);
            delay(MIN_MOTOR_SWITCH_TIME_MS);
        }
        digitalWrite(DRILL_MOTOR_PINS.direction, LOW);
        analogWrite(DRILL_MOTOR_PINS.enable, map(drill_speed, 0, 127, 0, 255));
    } else if (drill_speed < 0) {
        // drill reverse
        if (!isDigitalWrittenHigh(DRILL_MOTOR_PINS.direction)) {
            // shoot-through protection
            digitalWrite(DRILL_MOTOR_PINS.enable, LOW);
            delay(MIN_MOTOR_SWITCH_TIME_MS);
        }
        digitalWrite(DRILL_MOTOR_PINS.direction, HIGH);
        analogWrite(DRILL_MOTOR_PINS.enable, map(drill_speed, 0, -127, 0, 255));
    } else {
        // drill stop
        stopDrill();
    }
} 