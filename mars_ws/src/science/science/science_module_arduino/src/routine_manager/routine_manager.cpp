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
#define FUNC_TABLE_SIZE 2
#define ALL_ACTUATORS_AVAILABLE 0xFF
#define NO_FUNC 0xFF

#define DEBUG_ROUTINES

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
    void free_routine(routine_t* routine);
    void free_group(action_group_t* group_ptr);
    void report_routine_structure(routine_t* routine);

    // State variables
    const routine_t* active_routine;
    const action_group_t* active_group;
    uint8_t current_group_index;
    bool step_flag;

    // Actuator Control Methods
    uint8_t get_requested_actuators(const routine_t* routine);
    uint8_t get_requested_actuators(const action_group_t* group);
    uint8_t any_actuator_reserved(uint8_t bool_byte);
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

    template <typename T>
    T* push_newarray(T* array, uint8_t* size, T new_element) {
        T* new_array = (T*)calloc(*size + 1, sizeof(T)); // Reserve memory for an array one element bigger
        if (new_array == nullptr) { error::mallocError(); return nullptr; } // Failed to allocate memory, return nullptr
        if (*size > 0) memcpy(new_array, array, *size * sizeof(T)); // Copy over the old pointers
        new_array[*size] = new_element; // Add the new group
        (*size)++; // Increment the group count
        return new_array;
    }

    template <typename T>
    bool append(T** array_ptr, uint8_t* size_ptr, T new_element) {
        T* new_array = push_newarray<T>(*array_ptr, size_ptr, new_element); // Build a new array with the new element
        if (new_array == nullptr) return false; // Failed to allocate memory, return false
        if (*array_ptr != nullptr) free(*array_ptr); // Free the memory for the old array if it exists
        *array_ptr = new_array; // Set the new array to the pointer
        return true; // Addition was successful
    }

    bool all_actuators_available(uint8_t bool_byte) {
        uint8_t index;
        if ((index = any_actuator_reserved(bool_byte)) != ALL_ACTUATORS_AVAILABLE) {
            error::notReadyForRoutine(index);
            return false;
        }
        return true;
    }

    void init() {
        step_flag = false;
        routine_manager_state = IDLE;
    }

    bool begin_routine(routine_t* routine) {

        // Get the routine ptr
        active_routine = routine;

        // Reserve all now if required
        if (active_routine->lockdown_actuators) {
            uint8_t bool_byte = get_requested_actuators(active_routine);
            if (!all_actuators_available(bool_byte)) return false; // Throws error
            reserve_actuators(bool_byte);
        }
            
        // Get the first group
        current_group_index = 0;
        active_group = active_routine->group_ptrs[current_group_index];

        // Begin
        routine_manager_state = ADVANCE;
        return true;
    }

    void begin_routine(uint8_t routine_index) {
        routine_t* eeprom_routine = load_routine_from_eeprom(routine_index);
        if (eeprom_routine == nullptr) {
            if (response_buffer.error_code == ERROR_CODE_SUCCESS) {
                error::nullptrError(); // Not sure what happened
            }
            return;
        }
        if (!begin_routine(eeprom_routine)) // Start the routine
            free_routine(eeprom_routine); // Free if routine failed to start
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
                    && !all_actuators_available(get_requested_actuators(active_group))) { // Throws error 
                    abort();
                    break;
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
                if (active_group->func_index != NO_FUNC) {
                    #ifdef DEBUG_ROUTINES
                    Serial.print(F("Going to evaluate callback function #"));
                    Serial.println(active_group->func_index);
                    #endif
                    FUNC_TABLE[active_group->func_index]();
                }

                // Free all the actuators if it was a group reservation

                // Inc the routine group index
                if (++current_group_index == active_routine->group_cnt) {
                    // Complete the routine
                    end_routine();

                } else {
                    active_group = active_routine->group_ptrs[current_group_index];

                    #ifdef DEBUG_ROUTINES
                    Serial.print(F("moving to next group at "));
                    Serial.println((uint16_t)active_group);
                    #endif


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

        #ifdef DEBUG_ROUTINES
        Serial.println(F("ending routine!"));
        #endif

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

    // Check that the actuators are all available, returns 255 if all are available
    // else returns the first actuator that is reserved
    uint8_t any_actuator_reserved(uint8_t bool_byte) {

        // Check all these actuators are available
        for (uint8_t i = 0; i < LINEAR_ACTUATOR_CNT; i++) {
            if (((bool_byte >> i) & 0x01) == 0) continue;
            if (actuator_manager::is_reserved(i)) return i;
        }
        return ALL_ACTUATORS_AVAILABLE; // All actuators are available
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
            positional_controller::submit(pos_action_ptr->actuator_index, pos_action_ptr->position, pos_action_ptr->speed, active_routine->lockdown_actuators);
        }

        // Submit speed requests
        for (uint8_t i = 0; i < group->speed_action_cnt; i++) {
            const actuator_speed_action_t* speed_action_ptr = group->speed_action_ptrs[i];
            speed_controller::submit(speed_action_ptr->actuator_index, speed_action_ptr->control, speed_action_ptr->timeout, active_routine->lockdown_actuators);
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
        routine_t* routine_ptr = (routine_t*)calloc(1, sizeof(routine_t));
        if (routine_ptr == nullptr) { error::mallocError(); return nullptr; }
        #ifdef DEBUG_ROUTINES
        Serial.print(F("Loaded routine into 0x"));
        Serial.println((uint16_t)routine_ptr, HEX);
        #endif

        // State variables
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
                current_group_ptr = (action_group_t*)calloc(1, sizeof(action_group_t));
                if (current_group_ptr == nullptr) { error::mallocError(); break; }
                current_group_ptr->func_index = NO_FUNC; // init with no callback

                // Add this new group to the routine
                if (!append<action_group_t*>(&(routine_ptr->group_ptrs), &(routine_ptr->group_cnt), current_group_ptr)) {
                    // Handle a failed addition
                    free_group(current_group_ptr);
                    error::mallocError();
                    break;
                }

            } else if (instruction == POS_ACTION_CODE) {

                // Check the actuator index is valid
                uint8_t actuator_index = EEPROM.read(eeprom_ptr++); // Read next byte with index
                if (actuator_index >= LINEAR_ACTUATOR_CNT) {
                    // Handle a bad index
                    error::outOfBoundsError(actuator_index, LINEAR_ACTUATOR_CNT);
                    break;
                }

                // Reserve the memory a new position action
                actuator_position_action_t* pos_request_ptr = (actuator_position_action_t*)calloc(1, sizeof(actuator_position_action_t));
                if (pos_request_ptr == nullptr) { error::mallocError(); break; }

                // Init the struct members
                pos_request_ptr->actuator_index = actuator_index;
                pos_request_ptr->speed = 1.0f; // Full Speed
                pos_request_ptr->position = EEPROM.read(eeprom_ptr++) / 255.0f; // Read next byte with position into a float

                // Add this new position action to the group
                if (!append<actuator_position_action_t*>(&(current_group_ptr->pos_action_ptrs), &(current_group_ptr->pos_action_cnt), pos_request_ptr)) {
                    // Handle a failed addition
                    free(pos_request_ptr);
                    error::mallocError();
                    break;
                }

            } else if (instruction == SPD_ACTION_CODE) {

                // Check the actuator index is valid
                uint8_t actuator_index = EEPROM.read(eeprom_ptr++); // Read next byte with index
                if (actuator_index >= LINEAR_ACTUATOR_CNT) {
                    // Handle a bad index
                    error::outOfBoundsError(actuator_index, LINEAR_ACTUATOR_CNT);
                    break;
                }

                // Reserve the memory a new position action
                actuator_speed_action_t* speed_request_ptr = (actuator_speed_action_t*)calloc(1, sizeof(actuator_speed_action_t));
                if (speed_request_ptr == nullptr) { error::mallocError(); break; }

                // Init the struct members
                speed_request_ptr->actuator_index = actuator_index; // Read next byte with index
                speed_request_ptr->control = EEPROM.read(eeprom_ptr++); // Read next byte with control, cast into unsigned?
                speed_request_ptr->timeout = EEPROM_readObject<uint32_t>((uint32_t*)eeprom_ptr); // Read next 4 bytes for timeout
                eeprom_ptr += sizeof(uint32_t);

                // Add this new speed action to the group
                if (!append<actuator_speed_action_t*>(&(current_group_ptr->speed_action_ptrs), &(current_group_ptr->speed_action_cnt), speed_request_ptr)) {
                    // Handle a failed addition
                    free(speed_request_ptr);
                    error::mallocError();
                    break;
                }

            } else if (instruction == ACTUATOR_LOCKDOWN_CODE) {

                // Enable actuator lockdown on this routine,
                // this means that all actuators are only freed after the whole routine is done
                // and not after each group
                routine_ptr->lockdown_actuators = true; 

            } else if (instruction == FUNC_CODE) {

                // Sets the function pointer for this group
                // to one of the functions in the function table
                current_group_ptr->func_index = EEPROM.read(eeprom_ptr++);
                if (current_group_ptr->func_index >= FUNC_TABLE_SIZE) {
                    error::outOfBoundsError(current_group_ptr->func_index, FUNC_TABLE_SIZE);
                    break;
                }

            } else break; // Trigger the failed end of routine
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
        if (routine_index >= get_total_routine_count()) return nullptr;
        // Get the routine address from the lookup table but return it as a uint8_t*
        return (uint8_t*)EEPROM_readObject<uint16_t>((uint16_t*)(EEPROM_ROUTINE_LOOKUP_TABLE_ADDR) + routine_index);
    }

    // Free all the memory associated with a routine in RAM
    void free_routine(routine_t* routine) {

        #ifdef DEBUG_ROUTINES
        Serial.print(F("Freeing routine at 0x"));
        Serial.println((uint16_t)routine, HEX);
        #endif

        if (routine == nullptr) return;

        // Free each action group
        if (routine->group_ptrs != nullptr) {
            for (uint8_t i = 0; i < routine->group_cnt; i++) {
                free_group(routine->group_ptrs[i]);
            }
            // Free the group pointers array
            free(routine->group_ptrs);
        }

        // Free the routine itself
        free(routine);
    }

    void free_group(action_group_t* group_ptr) {
        if (group_ptr == nullptr) return;

        // Free positional actions
        if (group_ptr->pos_action_ptrs != nullptr) {
            for (uint8_t i = 0; i < group_ptr->pos_action_cnt; i++) {
                if (group_ptr->pos_action_ptrs[i] != nullptr)
                    free(group_ptr->pos_action_ptrs[i]);
            }
            free(group_ptr->pos_action_ptrs);
        }

        // Free speed actions
        if (group_ptr->speed_action_ptrs != nullptr) {
            for (uint8_t i = 0; i < group_ptr->speed_action_cnt; i++) {
                if (group_ptr->speed_action_ptrs[i] != nullptr)
                    free(group_ptr->speed_action_ptrs[i]);
            }
            free(group_ptr->speed_action_ptrs);
        }

        // Free the group itself
        free(group_ptr);
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
