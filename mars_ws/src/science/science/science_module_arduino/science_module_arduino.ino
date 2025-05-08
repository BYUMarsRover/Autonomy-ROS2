#include <Arduino.h>
#include "src/hardware/hardware.h"
#include "src/packet_comm/packet_comm.h"
#include "src/command_execution/command_execution.h"
#include "src/definitions/definitions.h"
#include "src/positional_controller/positional_controller.h"
#include "src/speed_controller/speed_controller.h"
#include "src/actuator_manager/actuator_manager.h"
#include "src/routine_manager/routine_manager.h"
#include "src/spectrograph/spectrograph.h"
#include "src/uv_sensor/uv_sensor.h"
#include "src/error/error.h"
#include "src/mem_manager/mem_manager.h"

#define ENABLE_I2C

unsigned long last_frame_timestamp;

unsigned long getDeltaTime() {
    unsigned long current_time = micros();
    unsigned long deltaTime = current_time - last_frame_timestamp;
    last_frame_timestamp = current_time;
    return deltaTime;
}

void setup() {
    // Init Hardware Communication
    setPinModes();
    stopAllActuators();

    // Init Serial Communication
    Serial.begin(BAUD_RATE);
    message::connection_successful();
    sendResponsePacket();


    // Start the timers
    last_frame_timestamp = micros();

    // Init state machines
    speed_controller::init();
    positional_controller::init();
    routine_manager::init();

    #ifdef ENABLE_I2C
    spectrograph::init();
    if (response_buffer.message_byte_len != 0) sendResponsePacket(); // Send connection status on bootup
    uv_sensor::init();
    if (response_buffer.message_byte_len != 0) sendResponsePacket(); // Send connection status on bootup
    #endif
}

void loop() {

    // Search for command packets
    if (findCommand()) {
        // Read command fields and verify
        if (loadCommand()) {
            executeCommand();
        }
    }

    unsigned long deltaTime = getDeltaTime();

    // Perform Integration
    actuator_manager::integrate(deltaTime);

    // Tick the speed controller
    speed_controller::tick(deltaTime);

    // Tick the positional controller
    positional_controller::tick();

    // Tick the routine manager
    routine_manager::tick();

    #ifdef ENABLE_I2C
    // Tick the spectrograph state machine
    spectrograph::tick(deltaTime);

    // Tick the uv index sensor state machine
    uv_sensor::tick();
    #endif

    if (shouldSendResponse()) {
        sendResponsePacket();
    }
}
