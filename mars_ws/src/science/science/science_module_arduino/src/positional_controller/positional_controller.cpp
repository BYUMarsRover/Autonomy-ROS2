#include "positional_controller.h"
#include "../error/error.h"
#include "../definitions/definitions.h"

#define MIN_SPEED 40
#define POS_FLOAT_TOL 1e-5

// #define DEBUG_POS_CONTROLLER

namespace positional_controller {
    
    linear_position_request_t pos_req_buffer[LINEAR_ACTUATOR_CNT];

    // Clears all positional requests
    void init() {
        for (uint8_t i = 0; i < LINEAR_ACTUATOR_CNT; i++)
            pos_req_buffer[i].resolved = true;
    };

    // Control all actutators that have an positional request
    void tick() {
        for (uint8_t i = 0; i < LINEAR_ACTUATOR_CNT; i++) {
            // Skip if already resolved
            linear_position_request_t* request = &(pos_req_buffer[i]);
            if (request->resolved == true) continue;

            linear_actuator_state_t state = actuator_state_array[i];
            float error = request->position - state.position;

            // Stop motion if paused
            if (request->paused) actuator_manager::set_control(i, 0);
            // Resolve if complete
            else if (abs(error) < POS_FLOAT_TOL) resolve(i);
            // Else update the control
            else {
                int8_t control = min(max(request->speed * 127, 0), 127) * (error < 0 ? -1 : 1);
                actuator_manager::set_control(i, control);
            }
        }
    }

    void configureDataAndStartControl(uint8_t actuator_index, float position, float speed, bool authorized) {
        // Configure data
        linear_position_request_t* request = &(pos_req_buffer[actuator_index]);
        request->position = position;
        request->speed = speed;
        request->authorized = authorized;
        request->paused = false;
        request->resolved = false;
    }

    // Updates a positional request 
    bool submit(uint8_t actuator_index, float position, float speed, bool authorized) {

        #ifdef DEBUG_POS_CONTROLLER
        Serial.print(F("Positional Controller Request actuator #"));
        Serial.print(actuator_index);
        Serial.print(F(" to position "));
        Serial.print(position);
        Serial.print(F(" at speed "));
        Serial.println(speed);
        #endif

        // Check if the actuator index is valid
        if (!verify::linear_actuator_index(actuator_index)) return false;

        // Reserve the actuator for control if not authorized
        if (!authorized && !actuator_manager::reserve(actuator_index)) return false;

        // If sucessful 
        configureDataAndStartControl(actuator_index, position, speed, authorized);
        return true;
    }

    // Updates a positional request, defaults to no auth
    bool submit(uint8_t actuator_index, float position, float speed) {
        return submit(actuator_index, position, speed, false);
    }

    bool is_resolved(uint8_t actuator_index) {
        return pos_req_buffer[actuator_index].resolved;
    }

    void set_paused(uint8_t actuator_index, bool paused) {
        pos_req_buffer[actuator_index].paused = paused;
    }

    void resolve(uint8_t index) {
        pos_req_buffer[index].resolved = true;
        actuator_manager::set_control(index, 0); // stop

        // Free if not authorized
        if (!pos_req_buffer[index].authorized) actuator_manager::free(index);
    };

    void report_to_message_buffer(uint8_t actuator_index) {
        linear_position_request_t request = pos_req_buffer[actuator_index];
        linear_actuator_state_t state = actuator_state_array[actuator_index];
        float error = request.position - state.position;

        float thou = abs(error * 1000);
        uint16_t start = floor(thou);
        uint16_t back = (thou - floor(thou)) * 1000;

        uint16_t len = snprintf_P(
            response_message_buffer,
            MAX_OPERAND_ARRAY_SIZE,
            (const char *)F("Positional Report Actuator[%d]:\n Resolved: %d\n Paused: %d\n Position: %d\n Speed: %d\n Error:%c%d.%d"),
            actuator_index,
            request.resolved,
            request.paused,
            uint8_t(request.position * 255),
            int8_t(request.speed * 127),
            (error < 0) ? '-' : ' ',
            start,
            back
        );
        response_buffer.error_code = ERROR_CODE_SUCCESS;
        response_buffer.message_byte_len = len;
    }

}
