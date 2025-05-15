#ifndef SPEED_CONTROLLER_H
#define SPEED_CONTROLLER_H

#include "../actuator_manager/actuator_manager.h"
#include <stdint.h> // Uint8_t Datatype

namespace speed_controller {

    struct speed_request_t {
        int8_t control;
        int32_t timeout;
        bool paused;
        bool resolved;
        bool authorized; // Normally false, set true to ignore reservation system
    };

    void init();
    void tick(unsigned long tick_period);
    bool submit(uint8_t actuator_index, int8_t control, int32_t timeout, bool authorized);
    bool submit(uint8_t actuator_index, int8_t control, int32_t timeout);
    bool is_resolved(uint8_t actuator_index);
    void set_paused(uint8_t actuator_index, bool paused);
    void resolve(uint8_t index);
    void resolve(uint8_t index, int8_t default_control);
    void report_to_message_buffer(uint8_t index);

}

#endif /* SPEED_CONTROLLER_H */
