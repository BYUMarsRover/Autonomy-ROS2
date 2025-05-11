#include "routine_manager.h"
#include "../positional_controller/positional_controller.h"
#include "../speed_controller/speed_controller.h"
#include "../actuator_manager/actuator_manager.h"
#include "../definitions/definitions.h"
#include "../error/error.h"
#include <stdint.h>

#define FULL_REVERSE_CONTROL 0x80
#define FULL_FORWARD_CONTROL 0x7F

namespace routine_manager {

    enum routine_manager_state_t {
        IDLE,
        ADVANCE,
        AWAITING_ACTUATORS,
        PAUSED_MID_ACTION,
        PAUSED_STEP,
    } routine_manager_state;

    const routine_t* active_routine;
    const action_group_t* active_group;
    uint8_t current_group_index;
    bool step_flag;

    uint8_t get_requested_actuators(const routine_t* routine);
    uint8_t get_requested_actuators(const action_group_t* group);
    bool all_actuators_available(uint8_t bool_byte);
    void reserve_actuators(uint8_t bool_byte);
    void free_actuators(uint8_t bool_byte);

    void submit_actuator_actions(const action_group_t* group);
    void pause_actuator_actions(const action_group_t* group);
    void resume_actuator_actions(const action_group_t* group);
    uint8_t unresolved_actions(const action_group_t* group);

    void init() {
        step_flag = false;
        routine_manager_state = IDLE;
    }

    void begin_routine(const routine_t* routine) {
        // Get the routine ptr
        active_routine = routine;

        // Reserve all now if required
        if (active_routine->lockdown_actuators) {
            uint8_t bool_byte = get_requested_actuators(active_routine);
            if (!all_actuators_available(bool_byte)) {
                active_routine = nullptr;
                // TODO use a proper error code
                Serial.println(F("**Error routine actuator reservation failed"));
                return;
            }
            reserve_actuators(bool_byte);
        }
            
        // Get the first group
        current_group_index = 0;
        active_group = active_routine->group_ptrs[current_group_index];

        // Begin
        routine_manager_state = ADVANCE;
    }

    void tick() {

        switch(routine_manager_state) {

            case IDLE:
            case PAUSED_STEP:
            case PAUSED_MID_ACTION:
                // Do nothing
                break;

            case ADVANCE:
            {

                // Ensure all actuators are available
                if (!active_routine->lockdown_actuators // Only check if the routine didn't take care of it
                    && !all_actuators_available(get_requested_actuators(active_group))) {
                    // Abort if reservation fails
                    // TODO use a proper error code
                    Serial.println(F("**Error group actuator reservation failed"));
                    routine_manager_state = IDLE;
                }

                // Submit all the positional requests
                submit_actuator_actions(active_group);

                // Move on to the waiting state
                routine_manager_state = AWAITING_ACTUATORS;
                break;
            }

            case AWAITING_ACTUATORS:
            {

                // Check to see if the action group is complete, break if not
                if (unresolved_actions(active_group) > 0) break;

                // Execute the func ptr if configured
                if (active_group->fun_ptr != nullptr) active_group->fun_ptr();

                // Free all the actuators if it was a group reservation

                // Inc the routine group index
                if (++current_group_index == active_routine->group_cnt) {
                    // Complete the routine

                    // Free Actuators is reserved by the routine
                    if (active_routine->lockdown_actuators)
                        free_actuators(get_requested_actuators(active_routine));

                    // End execution
                    active_routine = nullptr;
                    routine_manager_state = IDLE;

                } else {

                    active_group = active_routine->group_ptrs[current_group_index];

                    // Continue execution
                    if (step_flag) {
                        routine_manager_state = PAUSED_STEP;
                        step_flag = false; // consume flag
                    } else {
                        routine_manager_state = ADVANCE;
                    }
                }
                break;
            } 

            default:
                break;  
        }
    }

    // Abort the Routine
    void abort() {
        pause();
        routine_manager_state = IDLE;
    }

    // Pause the Routine
    void pause() {
        if (routine_manager_state == AWAITING_ACTUATORS) {
            pause_actuator_actions(active_group);
            routine_manager_state = PAUSED_MID_ACTION;
        } else if (routine_manager_state != PAUSED_MID_ACTION) {
            routine_manager_state = PAUSED_STEP;
        }
    }

    // Advance the routine by one step if paused
    // DO NOT CALL if not paused
    void step() {
        step_flag = true;
        resume();
    }

    // Resume normal operation if paused
    // DO NOT CALL if not paused
    void resume() {
        if (routine_manager_state == PAUSED_MID_ACTION) {
            resume_actuator_actions(active_group);
            routine_manager_state = AWAITING_ACTUATORS;
        } else {
            routine_manager_state = ADVANCE;
        }
    }

    uint8_t get_requested_actuators(const action_group_t* group) {
        uint8_t bool_byte = 0x00;
        for (uint8_t j = 0; j < group->pos_action_cnt; j++) {
            bool_byte = bool_byte | (0x01 << group->pos_action_ptrs[j]->actuator_index);
        }
        for (uint8_t j = 0; j < group->speed_action_cnt; j++) {
            bool_byte = bool_byte | (0x01 << group->speed_action_ptrs[j]->actuator_index);
        }
        return bool_byte;
    }

    uint8_t get_requested_actuators(const routine_t* routine) {
        uint8_t bool_byte = 0x00;
        for (uint8_t i = 0; i < routine->group_cnt; i++) {
            bool_byte = bool_byte | get_requested_actuators(routine->group_ptrs[i]);
        }
        return bool_byte;
    }

    // Check that the actuators are all available
    bool all_actuators_available(uint8_t bool_byte) {

        // Check all these actuators are available
        for (uint8_t i = 0; i < LINEAR_ACTUATOR_CNT; i++) {
            if (((bool_byte >> i) & 0x01) == 0) continue;
            if (actuator_manager::is_reserved(i)) return false;
        }
        return true;
    }

    // Reserve all requested actuators
    void reserve_actuators(uint8_t bool_byte) {

        // Iterate over byte and reserve
        for (uint8_t i = 0; i < LINEAR_ACTUATOR_CNT; i++) {
            if (((bool_byte >> i) & 0x01) == 0) continue;
            actuator_manager::reserve(i);
        }
    }

    // Free all requested actuators
    void free_actuators(uint8_t bool_byte) {

        // Iterate over byte and free
        for (uint8_t i = 0; i < LINEAR_ACTUATOR_CNT; i++) {
            if (((bool_byte >> i) & 0x01) == 0) continue;
            actuator_manager::free(i);
        }
    }

    // Submit actions in this group to the position controller
    void submit_actuator_actions(const action_group_t* group) {
        
        // Submit positional requests
        for (uint8_t i = 0; i < group->pos_action_cnt; i++) {
            const actuator_position_action_t* pos_action_ptr = group->pos_action_ptrs[i];
            positional_controller::submit(pos_action_ptr->actuator_index, pos_action_ptr->position, pos_action_ptr->speed, active_routine->lockdown_actuators, pos_action_ptr->fun_ptr);
        }

        // Submit speed requests
        for (uint8_t i = 0; i < group->speed_action_cnt; i++) {
            const actuator_speed_action_t* speed_action_ptr = group->speed_action_ptrs[i];
            speed_controller::submit(speed_action_ptr->actuator_index, speed_action_ptr->control, speed_action_ptr->timeout, active_routine->lockdown_actuators, speed_action_ptr->fun_ptr);
        }
    }

    void set_all_actuator_paused(const action_group_t* group, bool state) {

        // Position
        for (uint8_t i = 0; i < group->pos_action_cnt; i++) {
            const actuator_position_action_t* pos_action_ptr = group->pos_action_ptrs[i];
            positional_controller::set_paused(pos_action_ptr->actuator_index, state);
        }

        // Speed
        for (uint8_t i = 0; i < group->speed_action_cnt; i++) {
            const actuator_speed_action_t* speed_action_ptr = group->speed_action_ptrs[i];
            speed_controller::set_paused(speed_action_ptr->actuator_index, state);
        }
    }

    // Pauses the positional controller and speed controller by temporarily resolving their actions
    void pause_actuator_actions(const action_group_t* group) {
        set_all_actuator_paused(group, true);
    }

    // Resume the positional controller and speed controller by unresolving their actions
    void resume_actuator_actions(const action_group_t* group) {
        set_all_actuator_paused(group, false);
    }

    // Returns the number of incomplete actions
    uint8_t unresolved_actions(const action_group_t* group) {

        uint8_t cnt = 0;

        // Check positional actions
        for (uint8_t i = 0; i < group->pos_action_cnt; i++)
            if (!positional_controller::is_resolved(group->pos_action_ptrs[i]->actuator_index)) cnt++;

        // Check speed actions
        for (uint8_t i = 0; i < group->speed_action_cnt; i++)
            if (!speed_controller::is_resolved(group->speed_action_ptrs[i]->actuator_index)) cnt++;

        return cnt;
    }

    bool is_running() {
        return routine_manager_state != IDLE;
    }

    bool is_paused() {
        return routine_manager_state == PAUSED_MID_ACTION || routine_manager_state == PAUSED_STEP;
    }

    void report_to_message_buffer() {

        uint16_t len = snprintf_P(
            response_message_buffer,
            MAX_OPERAND_ARRAY_SIZE,
            (const char *)F("Routine Report:\n Current State: %d\nActive: %d\n Paused: %d\n Current Group: %d\n Unresolved Actions: %d"),
            routine_manager_state,
            is_running(),
            is_paused(),
            current_group_index,
            unresolved_actions(active_group)
        );
        response_buffer.error_code = ERROR_CODE_SUCCESS;
        response_buffer.message_byte_len = len;
    }

    namespace routines {

        // Move to Zero Routine
        // Moves all the actuators to zero position
        
        const actuator_position_action_t to_zero_auger = {
            .actuator_index = AUGER_ACTUATOR_INDEX,
            .position = 0,
            .speed = 1,
            .fun_ptr = nullptr
        };

        const actuator_position_action_t to_zero_probe = {
            .actuator_index = PROBE_ACTUATOR_INDEX,
            .position = 0,
            .speed = 1,
            .fun_ptr = nullptr
        };

        const actuator_position_action_t to_zero_primary_door = {
            .actuator_index = PRIMARY_DOOR_ACTUATOR_INDEX,
            .position = 0,
            .speed = 1,
            .fun_ptr = nullptr
        };

        const actuator_position_action_t to_zero_secondary_door = {
            .actuator_index = SECONDARY_DOOR_ACTUATOR_INDEX,
            .position = 0,
            .speed = 1,
            .fun_ptr = nullptr
        };

        const actuator_position_action_t to_zero_secondary_cache = {
            .actuator_index = SECONDARY_CACHE_ACTUATOR_INDEX,
            .position = 0,
            .speed = 1,
            .fun_ptr = nullptr
        };

        const actuator_position_action_t* zero_actions_addr[] = {
            &to_zero_auger,
            &to_zero_probe,
            &to_zero_primary_door,
            &to_zero_secondary_door,
            &to_zero_secondary_cache
        };

        const action_group_t move_to_zero_group = {
            .pos_action_ptrs = zero_actions_addr,
            .pos_action_cnt = 5,
            .speed_action_ptrs = nullptr,
            .speed_action_cnt = 0,
            .fun_ptr = nullptr
        };

        const action_group_t* zero_group_addr[] = {
            &move_to_zero_group
        };

        const routine_t move_to_zero_routine = {
            .group_ptrs = zero_group_addr,
            .group_cnt = 1,
            .lockdown_actuators = false
        };

        // Reset Routine
        // Retract all actuators long enough for them to be at zero

        const actuator_speed_action_t reset_probe = {
            .actuator_index = PROBE_ACTUATOR_INDEX,
            .control = FULL_REVERSE_CONTROL,
            .timeout = PROBE_RETRACT_TIME * 1e6,
            .fun_ptr = nullptr
        };

        const actuator_speed_action_t reset_auger = {
            .actuator_index = AUGER_ACTUATOR_INDEX,
            .control = FULL_REVERSE_CONTROL,
            .timeout = AUGER_RETRACT_TIME * 1e6,
            .fun_ptr = nullptr
        };

        const actuator_speed_action_t reset_primary_door = {
            .actuator_index = PRIMARY_DOOR_ACTUATOR_INDEX,
            .control = FULL_REVERSE_CONTROL,
            .timeout = PRIMARY_DOOR_RETRACT_TIME * 1e6,
            .fun_ptr = nullptr
        };

        const actuator_speed_action_t reset_secondary_door = {
            .actuator_index = SECONDARY_DOOR_ACTUATOR_INDEX,
            .control = FULL_REVERSE_CONTROL,
            .timeout = SECONDARY_DOOR_RETRACT_TIME * 1e6,
            .fun_ptr = nullptr
        };

        const actuator_speed_action_t reset_secondary_cache = {
            .actuator_index = SECONDARY_CACHE_ACTUATOR_INDEX,
            .control = FULL_REVERSE_CONTROL,
            .timeout = SECONDARY_CACHE_RETRACT_TIME * 1e6,
            .fun_ptr = nullptr
        };

        const actuator_speed_action_t* close_cache_actions_addr[] = {
            &reset_primary_door,
            &reset_secondary_door,
            &reset_secondary_cache
        };

        void zero_all_caches() {
            actuator_manager::set_position(PRIMARY_DOOR_ACTUATOR_INDEX, 0.0);
            actuator_manager::set_position(SECONDARY_DOOR_ACTUATOR_INDEX, 0.0);
            actuator_manager::set_position(SECONDARY_CACHE_ACTUATOR_INDEX, 0.0);
            #ifdef DEBUG
            Serial.println(F("Callback fired on caches"));
            #endif
        }

        const action_group_t close_caches_group = {
            .pos_action_ptrs = nullptr,
            .pos_action_cnt = 0,
            .speed_action_ptrs = close_cache_actions_addr,
            .speed_action_cnt = 3,
            .fun_ptr = zero_all_caches
        };

        const actuator_speed_action_t* retract_tools_addr[] = {
            &reset_probe,
            &reset_auger
        };

        void zero_all_tools() {
            actuator_manager::set_position(PROBE_ACTUATOR_INDEX, 0.0);
            actuator_manager::set_position(AUGER_ACTUATOR_INDEX, 0.0);
            #ifdef DEBUG
            Serial.println(F("Callback fired on tools"));
            #endif
        }

        const action_group_t retract_tools_group = {
            .pos_action_ptrs = nullptr,
            .pos_action_cnt = 0,
            .speed_action_ptrs = retract_tools_addr,
            .speed_action_cnt = 2,
            .fun_ptr = zero_all_tools
        };

        const action_group_t* reset_group_addr[] = {
            &close_caches_group,
            &retract_tools_group
        };

        const routine_t reset_routine = {
            .group_ptrs = reset_group_addr,
            .group_cnt = 2,
            .lockdown_actuators = false
        };

        // Test Routine

        const routine_t test_routine = {
            .group_ptrs = reset_group_addr,
            .group_cnt = 2,
            .lockdown_actuators = true
        }; 

        // First Cache Transfer Routine

        const actuator_speed_action_t align_auger_first_cache_transfer = {
            .actuator_index = AUGER_ACTUATOR_INDEX,
            .control = FULL_FORWARD_CONTROL,
            .timeout = 1350,
            .fun_ptr = nullptr
        };

        const actuator_speed_action_t align_secondary_first_cache_transfer = {
            .actuator_index = SECONDARY_CACHE_ACTUATOR_INDEX,
            .control = FULL_FORWARD_CONTROL,
            .timeout = 9000,
            .fun_ptr = nullptr
        };

        const actuator_speed_action_t* align_first_cache_action_ptrs[] = {
            &align_auger_first_cache_transfer,
            &align_secondary_first_cache_transfer
        };

        const action_group_t align_first_cache_group = {
            .pos_action_ptrs = nullptr,
            .pos_action_cnt = 0,
            .speed_action_ptrs = align_first_cache_action_ptrs,
            .speed_action_cnt = 2,
            .fun_ptr = nullptr
        };

        const action_group_t* align_first_cache_group_addr[] = {
            &align_first_cache_group
        };

        const routine_t first_cache_transfer = {
            .group_ptrs = align_first_cache_group_addr,
            .group_cnt = 1,
            .lockdown_actuators = true
        }; 
    }
}

extern const routine_manager::routine_t* ROUTINE_LOOKUP[ROUTINE_COUNT] = {
    &routine_manager::routines::reset_routine,
    &routine_manager::routines::move_to_zero_routine,
    &routine_manager::routines::test_routine,
    &routine_manager::routines::first_cache_transfer
};
