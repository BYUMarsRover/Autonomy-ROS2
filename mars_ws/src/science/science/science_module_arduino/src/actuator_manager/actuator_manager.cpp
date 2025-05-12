#include "actuator_manager.h"
#include "../definitions/definitions.h"
#include "../hardware/hardware.h"
#include "../print_func/print_func.h"

#include <stdint.h>

namespace actuator_manager {

    uint8_t actuator_check_out_byte;

    bool reserve(uint8_t actuator_index) {
        if (is_reserved(actuator_index))
            // Fail if already checked out
            return false;

        // Set the bit at the actuator_index
        actuator_check_out_byte = actuator_check_out_byte | (0b00000001 << actuator_index);

        #ifdef DEBUG
        Serial.print(F("Reserve("));
        Serial.print(actuator_index);
        Serial.println(F(")"));
        #endif

        return true;
    }

    void free(uint8_t actuator_index) {
        // Reset the bit at the actuator_index
        actuator_check_out_byte = actuator_check_out_byte & ~(0b00000001 << actuator_index);

        #ifdef DEBUG
        Serial.print(F("Free("));
        Serial.print(actuator_index);
        Serial.println(F(")"));
        #endif
    }

    bool is_reserved(uint8_t actuator_index) {
        bool v = (actuator_check_out_byte & (0b00000001 << actuator_index)) > 0;
        return v;
    }

    void integrate(unsigned long deltaTime) {
        // deltaTime - Time since last intergration step in microseconds
        // Integrate the speed into position
        for (uint8_t i = 0; i < LINEAR_ACTUATOR_CNT; i++) {
            linear_actuator_pins_t pins = ACTUATOR_PIN_ARRAY[i];
            linear_actuator_state_t state = actuator_state_array[i];
            float speed = 1 / ((state.control > 0) ? pins.extend_rate : -pins.retract_rate);
            float delta = speed * (abs(state.control) / 127.0f) * (deltaTime / 1.0e6f);
            actuator_state_array[i].position = min(max(state.position + delta, 0.0f), 1.0f);
        }
    }

    uint8_t get_position(uint8_t actuator_index) {
        return uint8_t(round(actuator_state_array[actuator_index].position * 255.0f));
    }

    void set_position(uint8_t actuator_index, uint8_t pos_byte) {
        actuator_state_array[actuator_index].position = (pos_byte / 255.0f);
    }

    int8_t get_control(uint8_t actuator_index) {
        return actuator_state_array[actuator_index].control;
    }

    void set_control(uint8_t actuator_index, int8_t control_byte) {
        actuator_state_array[actuator_index].control = control_byte;
        writeActuator(actuator_index, control_byte);
    }

    void set_drill_control(uint8_t control_byte) {
        writeDrill(control_byte);
    }

}