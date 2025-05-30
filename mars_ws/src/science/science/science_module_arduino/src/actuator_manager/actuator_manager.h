#ifndef ACTUATOR_MANAGER_H
#define ACTUATOR_MANAGER_H

#include <stdint.h> // Uint8_t Datatype

namespace actuator_manager {

    bool reserve(uint8_t actuator_index);
    void free(uint8_t actuator_index);
    bool is_reserved(uint8_t actuator_index);

    void integrate(unsigned long deltaTime);
    uint8_t get_position(uint8_t actuator_index);
    void set_position(uint8_t actuator_index, uint8_t state_byte);
    int8_t get_control(uint8_t actuator_index);
    void set_control(uint8_t actuator_index, int8_t control_byte);
    void set_drill_control(uint8_t control_byte);

}

#endif /* ACTUATOR_MANAGER_H */
