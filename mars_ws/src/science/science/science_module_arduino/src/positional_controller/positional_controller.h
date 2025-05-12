#ifndef POSITIONAL_CONTROLLER_H
#define POSITIONAL_CONTROLLER_H

#include "../actuator_manager/actuator_manager.h"
#include <stdint.h> // Uint8_t Datatype

namespace positional_controller {

    struct linear_position_request_t {
        float position;
        float speed; // 0 - 1 Slowest Speed to Highest speed
        bool paused;
        bool authorized; // Normally false, set true to ignore reservation system
        bool resolved;
        void (*callback)(void);
    };

    void init();
    void tick();
    bool submit(uint8_t actuator_index, float position, float speed);
    bool submit(uint8_t actuator_index, float position, float speed, bool authorized, void (*callback)(void));
    bool is_resolved(uint8_t actuator_index);
    void set_paused(uint8_t actuator_index, bool paused);
    void resolve(uint8_t index);
    void report_to_message_buffer(uint8_t index);

}

#endif /* POSITIONAL_CONTROLLER_H */
