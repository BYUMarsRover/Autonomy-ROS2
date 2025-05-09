#include "speed_controller.h"
#include "../definitions/definitions.h"
#include "../error/error.h"

#define DRILL_INDEX LINEAR_ACTUATOR_CNT

namespace speed_controller {
    
    // +1 to also hold the drill
    speed_request_t speed_req_buffer[LINEAR_ACTUATOR_CNT + 1];

    // Clears all speed requests
    void init() {
        for (uint8_t i = 0; i < LINEAR_ACTUATOR_CNT; i++)
            speed_req_buffer[i].resolved = true;
    };

    // Control all actutators that have an speed request
    void tick(unsigned long tick_period) {
        for (uint8_t i = 0; i < LINEAR_ACTUATOR_CNT; i++) {

            // Skip if already resolved
            speed_request_t* request = &(speed_req_buffer[i]);
            if (request->resolved == true) continue;

            // Resolve if complete
            if (request->timeout <= 0) {

                // Stop the actuator if the time completed
                // if the time is negative, it was meant to stay on
                resolve(i, (request->timeout == 0) ? 0 : request->control);

            // Else count time
            } else {
                request->timeout = request->timeout - (tick_period / 1e3);
                if (request->timeout < 0) request->timeout = 0;
            }
        }
    }

    void configureDataAndStartControl(uint8_t actuator_index, int8_t control, int32_t timeout, bool authorized, void (*callback)(void)) {
        // Configure Data
        speed_request_t* request = &(speed_req_buffer[actuator_index]);
        request->control = control;
        request->timeout = timeout;
        request->resolved = false;
        request->authorized = authorized;
        request->callback = callback;

        // Start send the control
        if (actuator_index == DRILL_INDEX) {
            actuator_manager::set_drill_control(control);
        } else actuator_manager::set_control(actuator_index, control);
    }

    // Updates a speed request 
    bool submit(uint8_t actuator_index, int8_t control, int32_t timeout, bool authorized, const void (*callback)(void)) {
        // Reserve the actuator for control if not authorized
        if (!authorized && !actuator_manager::reserve(actuator_index)) return false;

        // If sucessful 
        configureDataAndStartControl(actuator_index, control, timeout, authorized, callback);
        return true;
    }

    // Updates a speed requests, defaults to no auth or a callback
    bool submit(uint8_t actuator_index, int8_t control, int32_t timeout) {
        return submit(actuator_index, control, timeout, false, nullptr);
    }

    bool is_resolved(uint8_t actuator_index) {
        return speed_req_buffer[actuator_index].resolved;
    }

    void set_paused(uint8_t actuator_index, bool paused) {
        speed_req_buffer[actuator_index].paused = paused;
    }

    void resolve(uint8_t actuator_index) {
        resolve(actuator_index, 0);
    };

    void resolve(uint8_t index, int8_t default_control) {
        #ifdef DEBUG
        Serial.print(F("Speed Controller Resolved actuator #"));
        Serial.println(index);
        #endif
        speed_req_buffer[index].resolved = true;

        // Execute callback on resolution
        if (speed_req_buffer[index].callback != nullptr) speed_req_buffer[index].callback();

        // Stop the control
        if (index == DRILL_INDEX) {
            actuator_manager::set_drill_control(default_control);
        } else actuator_manager::set_control(index, default_control);

        // If not authorized free the actuator
        if (!speed_req_buffer[index].authorized) actuator_manager::free(index);
    };

    void report_to_message_buffer(uint8_t actuator_index) {
        speed_request_t request = speed_req_buffer[actuator_index];
        uint16_t len = snprintf_P(
            response_message_buffer,
            MAX_OPERAND_ARRAY_SIZE,
            (const char *)F("Speed Report Actuator[%d]:\n Resolved: %d\n Paused: %d\n Control: %d\n Timeout: %ld"),
            actuator_index,
            request.resolved,
            request.paused,
            request.control,
            request.timeout
        );
        response_buffer.error_code = ERROR_CODE_SUCCESS;
        response_buffer.message_byte_len = len;
    }

}
