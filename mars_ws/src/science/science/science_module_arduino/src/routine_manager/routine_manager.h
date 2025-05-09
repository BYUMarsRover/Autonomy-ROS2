#ifndef ROUTINE_MANAGER_H
#define ROUTINE_MANAGER_H

#include <stdint.h>

#define ROUTINE_COUNT 4

namespace routine_manager {

    struct actuator_position_action_t {
        const uint8_t actuator_index;
        const float position;
        const float speed;
        const void (*fun_ptr)(void);
    };

    struct actuator_speed_action_t {
        const uint8_t actuator_index;
        const int8_t control;
        const int32_t timeout;
        const void (*fun_ptr)(void);
    };

    struct action_group_t {
        const actuator_position_action_t** pos_action_ptrs;
        const uint8_t pos_action_cnt;
        const actuator_speed_action_t** speed_action_ptrs;
        const uint8_t speed_action_cnt;
        const void (*fun_ptr)(void);
    };

    // Array of Action Groups
    struct routine_t {
        const action_group_t** group_ptrs;
        const uint8_t group_cnt;
        const bool lockdown_actuators;
    };

    void begin_routine(routine_t* routine);
    void init();
    void tick();
    void step();
    void pause();
    void abort();
    void resume();
    bool is_running();
    bool is_paused();
    void report_to_message_buffer();

    namespace routines {
        extern const routine_t move_to_zero_routine;
        extern const routine_t reset_routine;
        extern const routine_t test_routine;
        extern const routine_t first_cache_transfer;
    }
}

extern const routine_manager::routine_t* ROUTINE_LOOKUP[ROUTINE_COUNT];

#endif /* ROUTINE_MANAGER_H */
