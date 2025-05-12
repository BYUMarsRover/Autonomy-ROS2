#ifndef COMMAND_EXECUTION_H
#define COMMAND_EXECUTION_H

#include <stdint.h>
#include <Arduino.h>

void executeCommand();
void executeAction(uint8_t function_addr, uint8_t override);
void executeQuery(uint8_t function_addr, uint8_t override);

#endif /* COMMAND_EXECUTION_H */