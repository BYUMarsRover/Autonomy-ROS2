#include "actuator_manager.h"
#include "../definitions/definitions.h"
#include "../hardware/hardware.h"
#include "../print_func/print_func.h"
#include "../mem_manager/mem_map.h"
#include "../mem_manager/mem_manager.h"

#include <stdint.h>

//#define DEBUG_RESERVATION

namespace actuator_manager {

    float get_extend_rate(uint8_t actuator_index);
    float get_retract_rate(uint8_t actuator_index);
    float get_extend_rate_eeprom(uint8_t actuator_index);
    float get_retract_rate_eeprom(uint8_t actuator_index);

    uint8_t actuator_check_out_byte;

    bool reserve(uint8_t actuator_index) { 
        if (actuator_index >= TOTAL_ACTUATOR_CNT) return false;
        if (is_reserved(actuator_index))
            // Fail if already checked out
            return false;

        // Set the bit at the actuator_index
        actuator_check_out_byte = actuator_check_out_byte | (0b00000001 << actuator_index);

        #ifdef DEBUG_RESERVATION
        Serial.print(F("Reserve("));
        Serial.print(actuator_index);
        Serial.println(F(")"));
        #endif

        return true;
    }

    void free(uint8_t actuator_index) {
        if (actuator_index >= TOTAL_ACTUATOR_CNT) return;
        // Reset the bit at the actuator_index
        actuator_check_out_byte = actuator_check_out_byte & ~(0b00000001 << actuator_index);

        #ifdef DEBUG_RESERVATION
        Serial.print(F("Free("));
        Serial.print(actuator_index);
        Serial.println(F(")"));
        #endif
    }

    bool is_reserved(uint8_t actuator_index) {
        if (actuator_index >= TOTAL_ACTUATOR_CNT) return false;
        bool v = (actuator_check_out_byte & (0b00000001 << actuator_index)) > 0;
        return v;
    }

    void integrate(unsigned long deltaTime_mu) {
        // deltaTime_mu - Time since last intergration step in microseconds
        // Integrate the speed into position
        for (uint8_t i = 0; i < LINEAR_ACTUATOR_CNT; i++) {
            linear_actuator_pins_t pins = ACTUATOR_PIN_ARRAY[i];
            linear_actuator_state_t state = actuator_state_array[i];
            float speed = 1 / ((state.control > 0) ? get_extend_rate(i) : -get_retract_rate(i));
            float delta = speed * (abs(state.control) / 127.0f) * (deltaTime_mu / 1.0e6f);
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

    float get_extend_rate(uint8_t actuator_index) {
        if (actuator_speed_array[actuator_index].extend_rate_ms == 0.0f) {
            // If the extend rate is not set, read it from EEPROM
            actuator_speed_array[actuator_index].extend_rate_ms = get_extend_rate_eeprom(actuator_index);
        }
        return actuator_speed_array[actuator_index].extend_rate_ms;
    }

    float get_retract_rate(uint8_t actuator_index) {
        if (actuator_speed_array[actuator_index].retract_rate_ms == 0.0f) {
            // If the extend rate is not set, read it from EEPROM
            actuator_speed_array[actuator_index].retract_rate_ms = get_retract_rate_eeprom(actuator_index);
        }
        return actuator_speed_array[actuator_index].retract_rate_ms;
    }

    float get_extend_rate_eeprom(uint8_t actuator_index) {
        // Serial.print(F("Load extend rate from EEPROM for actuator "));
        // Serial.print(actuator_index);
        return EEPROM_readObject<float>((float*)(EEPROM_ACTUATOR_EXTEND_RETRACT_TIME_TABLE_ADDR) + (actuator_index * 2));
    }

    float get_retract_rate_eeprom(uint8_t actuator_index) {
        // Serial.print(F("Load retract rate from EEPROM for actuator "));
        // Serial.print(actuator_index);
        return EEPROM_readObject<float>((float*)(EEPROM_ACTUATOR_EXTEND_RETRACT_TIME_TABLE_ADDR) + (actuator_index * 2) + 1);
    }

}