#include "command_execution.h"
#include "../definitions/definitions.h"
#include <Arduino.h>

void executeCommand() {
    // Check if the command is a command or a query
    bool isAction = (command_buffer.read_command_word & 0b10000000);
    uint8_t function_addr = (command_buffer.read_command_word & 0b00011111);
    bool overrideEnabled = (command_buffer.read_command_word & 0b01000000);

    // Branch depending on the action type
    if (isAction) executeAction(function_addr, overrideEnabled);
    else executeQuery(function_addr, overrideEnabled);
}