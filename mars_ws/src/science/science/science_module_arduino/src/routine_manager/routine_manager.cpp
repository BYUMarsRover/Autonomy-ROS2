#include "routine_manager.h"
#include "../positional_controller/positional_controller.h"
#include "../speed_controller/speed_controller.h"
#include "../actuator_manager/actuator_manager.h"
#include "../definitions/definitions.h"
#include "../error/error.h"
#include "../mem_manager/mem_map.h"
#include "../mem_manager/mem_manager.h"
#include <stdint.h>

#define FULL_REVERSE_CONTROL 0x80
#define FULL_FORWARD_CONTROL 0x7F

#define GROUP_CODE 0xEE
#define END_OF_ROUTINE 0xFF
#define POS_ACTION_CODE 0xAA
#define SPD_ACTION_CODE 0xBB
#define ACTUATOR_LOCKDOWN_CODE 0xCC
#define FUNC_CODE 0xDD

namespace routine_manager {

    enum routine_manager_state_t {
        IDLE,
        ADVANCE,
        AWAITING_ACTUATORS,
        PAUSED_MID_ACTION,
        PAUSED_STEP,
    } routine_manager_state;

    // Memory manipulation Methods
    routine_t* load_routine_from_eeprom(uint8_t routine_index);
    uint8_t* get_routine_ptr_from_eeprom(uint8_t routine_index);
    void add_action_group_to_routine(routine_t* routine, action_group_t* new_group);
    void add_pos_action_to_group(action_group_t* group, actuator_position_action_t* new_pos_action);
    void add_speed_action_to_group(action_group_t* group, actuator_speed_action_t* new_speed_action);
    void free_routine(routine_t* routine);
    void report_routine_structure(routine_t* routine);

    // State variables
    const routine_t* active_routine;
    const action_group_t* active_group;
    uint8_t current_group_index;
    bool step_flag;

    // Actuator Control Methods
    uint8_t get_requested_actuators(const routine_t* routine);
    uint8_t get_requested_actuators(const action_group_t* group);
    bool all_actuators_available(uint8_t bool_byte);
    void reserve_actuators(uint8_t bool_byte);
    void free_actuators(uint8_t bool_byte);
    void end_routine();
    void submit_actuator_actions(const action_group_t* group);
    void pause_actuator_actions(const action_group_t* group);
    void resume_actuator_actions(const action_group_t* group);
    uint8_t unresolved_actions(const action_group_t* group);

    // Callback Functions
    void zero_all_caches();
    void zero_all_tools();
    void (*FUNC_TABLE[])() = {
        zero_all_caches,
        zero_all_tools
    };

    void init() {
        step_flag = false;
        routine_manager_state = IDLE;
    }

    void begin_routine(routine_t* routine) {

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

    void begin_routine(uint8_t routine_index) {
        routine_t* eeprom_routine = load_routine_from_eeprom(routine_index);
        if (eeprom_routine == nullptr) {
            error::nullptrError();
            return;
        }
        Serial.println(F("Routine loaded from EEPROM:"));
        report_routine_structure(eeprom_routine);
        Serial.println(F("Beginning routine:"));
        begin_routine(eeprom_routine);
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
                if (active_group->fun_ptr != nullptr) {
                    Serial.println(F("Going to evaluate callback!"));
                    active_group->fun_ptr();
                }

                // Free all the actuators if it was a group reservation

                // Inc the routine group index
                if (++current_group_index == active_routine->group_cnt) {
                    // Complete the routine
                    end_routine();

                } else {
                    Serial.print(F("moving to next group at "));
                    active_group = active_routine->group_ptrs[current_group_index];
                    Serial.println((uint16_t)active_group);


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

    void end_routine() {
        Serial.println(F("ending routine!"));

        // Free all the actuators if it was a group reservation
        if (active_routine->lockdown_actuators)
            free_actuators(get_requested_actuators(active_routine));

        // End execution
        free_routine(active_routine);
        active_routine = nullptr;
        routine_manager_state = IDLE;
    }

    // Abort the Routine
    void abort() {
        pause();
        end_routine();
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

    void report_routine_structure(routine_t* routine) {
        Serial.print(F("Routine Structure:"));
        Serial.print(F("Routine at "));
        Serial.println((uint16_t)routine);
        Serial.print(F("Routine Group Count: "));
        Serial.println(routine->group_cnt);
        for (uint8_t i = 0; i < routine->group_cnt; i++) {
            Serial.print(F("Group "));
            Serial.print(i);
            Serial.print(F(": "));
            Serial.println((uint16_t)routine->group_ptrs[i]);
            if (routine->group_ptrs[i]->pos_action_cnt == 0) {
                Serial.println(F("No Positional Actions"));
            } else {
                Serial.print(routine->group_ptrs[i]->pos_action_cnt);
                Serial.print(F(" Pos Actions configured"));
                for( uint8_t j = 0; j < routine->group_ptrs[i]->pos_action_cnt; j++) {
                    Serial.print(F("Actuator "));
                    Serial.print(routine->group_ptrs[i]->pos_action_ptrs[j]->actuator_index);
                    Serial.print(F(" to "));
                    Serial.print(routine->group_ptrs[i]->pos_action_ptrs[j]->position);
                    Serial.print(F(" at speed "));
                    Serial.println(routine->group_ptrs[i]->pos_action_ptrs[j]->speed);
                }
            }
            if (routine->group_ptrs[i]->speed_action_cnt == 0) {
                Serial.println(F("No Speed Actions"));
            } else {
                Serial.print(routine->group_ptrs[i]->speed_action_cnt);
                Serial.print(F(" Speed Actions configured"));
                for( uint8_t j = 0; j < routine->group_ptrs[i]->speed_action_cnt; j++) {
                    Serial.print(F("Actuator "));
                    Serial.print(routine->group_ptrs[i]->speed_action_ptrs[j]->actuator_index);
                    Serial.print(F(" to "));
                    Serial.println(routine->group_ptrs[i]->speed_action_ptrs[j]->control);
                }
            }
        }
    }

    // Builds a routine data object from EEPROM
    routine_t* load_routine_from_eeprom(uint8_t routine_index) {
        // Load the routine data into RAM from eeprom
        uint8_t* eeprom_ptr = get_routine_ptr_from_eeprom(routine_index);

        // Reserve the memory for the routine, set up handles
        routine_t* routine_ptr = (routine_t*)malloc(sizeof(routine_t));
        if (routine_ptr == nullptr) { error::mallocError(); return nullptr; }

        // Init the struct members
        routine_ptr->group_cnt = 0;
        routine_ptr->lockdown_actuators = nullptr;
        action_group_t* current_group_ptr;

        // Read from EEPROM
        uint8_t instruction;
        while (eeprom_ptr <= MAX_EEPROM_ADDR) {

            // Read the next byte from EEPROM
            instruction = EEPROM.read(eeprom_ptr++);

            if (instruction == END_OF_ROUTINE) {
                // End of routine, clean up data and return ptr to the routine
                break;

            } else if (instruction == GROUP_CODE) {

                // Reserve the memory for the new group
                current_group_ptr = (action_group_t*)malloc(sizeof(action_group_t));
                if (current_group_ptr == nullptr) { error::mallocError(); return nullptr; }

                // Init the struct members 
                current_group_ptr->pos_action_cnt = 0;
                current_group_ptr->speed_action_cnt = 0;
                current_group_ptr->fun_ptr = nullptr;

                // Add this new group to the routine
                add_action_group_to_routine(routine_ptr, current_group_ptr);

            } else if (instruction == POS_ACTION_CODE) {

                // Reserve the memory a new position action
                actuator_position_action_t* pos_request_ptr = (actuator_position_action_t*)malloc(sizeof(actuator_position_action_t));
                if (routine_ptr == nullptr) { error::mallocError(); return nullptr; }

                // Init the struct members 
                pos_request_ptr->actuator_index = EEPROM.read(eeprom_ptr++); // Read next byte with index
                pos_request_ptr->speed = 1.0f; // Full Speed
                pos_request_ptr->position = EEPROM.read(eeprom_ptr++) / 255.0f; // Read next byte with position into a float
                pos_request_ptr->fun_ptr = nullptr; // No function pointer

                // Add this new position action to the group
                add_pos_action_to_group(current_group_ptr, pos_request_ptr);

            } else if (instruction == SPD_ACTION_CODE) {

                // Reserve the memory a new position action
                actuator_speed_action_t* speed_request_ptr = (actuator_speed_action_t*)malloc(sizeof(actuator_speed_action_t));
                if (speed_request_ptr == nullptr) { error::mallocError(); return nullptr; }

                // Init the struct members
                speed_request_ptr->actuator_index = EEPROM.read(eeprom_ptr++); // Read next byte with index
                speed_request_ptr->control = EEPROM.read(eeprom_ptr++); // Read next byte with control, cast into unsigned?
                speed_request_ptr->timeout = EEPROM_readObject<uint32_t>((uint32_t*)eeprom_ptr); // Read next 4 bytes for timeout
                eeprom_ptr += sizeof(uint32_t);
                speed_request_ptr->fun_ptr = nullptr; // No function pointer

                // Add this new position action to the group
                add_speed_action_to_group(current_group_ptr, speed_request_ptr);

            } else if (instruction == ACTUATOR_LOCKDOWN_CODE) {

                // Enable actuator lockdown on this routine,
                // this means that all actuators are only freed after the whole routine is done
                // and not after each group
                routine_ptr->lockdown_actuators = true; 

            } else if (instruction == FUNC_CODE) {

                // Sets the function pointer for this group
                // to one of the functions in the function table
                current_group_ptr->fun_ptr = FUNC_TABLE[EEPROM.read(eeprom_ptr++)]; // Read next byte with function pointer

            }
        }

        // Check if routine loading was successful
        if (instruction != END_OF_ROUTINE) {
            // Free partially allocated memory in case of an error
            free_routine(routine_ptr);
            return nullptr;
        }

        // Return the new routine
        return routine_ptr;
    }

    uint8_t get_total_routine_count() {
        // Get the total number of routines
        return EEPROM.read(EEPROM_ROUTINE_LOOKUP_TABLE_SIZE_ADDR);
    }

    uint8_t* get_routine_ptr_from_eeprom(uint8_t routine_index) {
        // Get the ptr to the specified routine
        // Ensure the routine index is within the valid range of routines stored in the EEPROM
        if (routine_index >= get_total_routine_count()) {
            return nullptr;
        }
        // Get the routine address from the lookup table but return it as a uint8_t*
        return (uint8_t*)EEPROM_readObject<uint16_t>((uint16_t*)(EEPROM_ROUTINE_LOOKUP_TABLE_ADDR) + routine_index);
    }

    // Dynamically add an action_group to a routine object
    void add_action_group_to_routine(routine_t* routine, action_group_t* new_group) {
        // Reserve memory for a routine one group bigger
        action_group_t** new_action_group_array = (action_group_t**)malloc(sizeof(action_group_t*) * (routine->group_cnt + 1));

        // Handle moving prev data
        if (routine->group_cnt > 0) {
            // Copy over the old pointers
            memcpy(new_action_group_array, routine->group_ptrs, routine->group_cnt * sizeof(action_group_t*));
            // Free the memory for the old array
            free(routine->group_ptrs);
        }

        // Add the new group and increment the group cnt
        new_action_group_array[routine->group_cnt++] = new_group;
        // Attach the new array to the routine
        routine->group_ptrs = new_action_group_array;
    }

    // Dynamically add an position action to an action group
    void add_pos_action_to_group(action_group_t* group, actuator_position_action_t* new_pos_action) {
        // Reserve memory for a group one pos action bigger
        actuator_position_action_t** new_pos_action_array = (actuator_position_action_t**)malloc(sizeof(actuator_position_action_t*) * (group->pos_action_cnt + 1));

        // Handle moving prev data
        if (group->pos_action_cnt > 0) {
            // Copy over the old pointers
            memcpy(new_pos_action_array, group->pos_action_ptrs, group->pos_action_cnt * sizeof(actuator_position_action_t*));
            // Free the memory for the old array
            free(group->pos_action_ptrs);
        }

        // Add the new group and increment the group cnt
        new_pos_action_array[group->pos_action_cnt++] = new_pos_action;
        // Attach the new array to the routine
        group->pos_action_ptrs = new_pos_action_array;
    }

    // Dynamically add a speed action to an action group
    void add_speed_action_to_group(action_group_t* group, actuator_speed_action_t* new_speed_action) {
        // Reserve memory for a group one speed action bigger
        actuator_speed_action_t** new_speed_action_array = (actuator_speed_action_t**)malloc(sizeof(actuator_speed_action_t*) * (group->speed_action_cnt + 1));

        // Handle moving prev data
        if (group->speed_action_cnt > 0) {
            // Copy over the old pointers
            memcpy(new_speed_action_array, group->speed_action_ptrs, group->speed_action_cnt * sizeof(actuator_speed_action_t*));
            // Free the memory for the old array
            free(group->speed_action_ptrs);
        }

        // Add the new action and increment the action count
        new_speed_action_array[group->speed_action_cnt++] = new_speed_action;
        // Attach the new array to the group
        group->speed_action_ptrs = new_speed_action_array;
    }

    // Free all the memory associated with a routine in RAM
    void free_routine(routine_t* routine) {
        if (routine == nullptr) return;

        Serial.print(F("Freeing routine at "));
        Serial.println((uint16_t)routine);

        report_routine_structure(routine);

        // Free each action group
        for (uint8_t i = 0; i < routine->group_cnt; i++) {
            action_group_t* group = routine->group_ptrs[i];
            if (group) {
                // Free positional actions
                for (uint8_t j = 0; j < group->pos_action_cnt; j++) {
                    free(group->pos_action_ptrs[j]);
                }
                free(group->pos_action_ptrs);

                // Free speed actions
                for (uint8_t j = 0; j < group->speed_action_cnt; j++) {
                    free(group->speed_action_ptrs[j]);
                }
                free(group->speed_action_ptrs);

                // Free the group itself
                free(group);
            }
        }

        // Free the group pointers array
        free(routine->group_ptrs);

        // Free the routine itself
        free(routine);
    }

    void zero_all_caches() {
        actuator_manager::set_position(PRIMARY_DOOR_ACTUATOR_INDEX, 0.0);
        actuator_manager::set_position(SECONDARY_DOOR_ACTUATOR_INDEX, 0.0);
        actuator_manager::set_position(SECONDARY_CACHE_ACTUATOR_INDEX, 0.0);
    }

    void zero_all_tools() {
        actuator_manager::set_position(PROBE_ACTUATOR_INDEX, 0.0);
        actuator_manager::set_position(AUGER_ACTUATOR_INDEX, 0.0);
    }

}
