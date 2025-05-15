#ifndef PACKET_COMM_H
#define PACKET_COMM_H

#include <stdint.h>
#include <Arduino.h>

void writeToMessageBuffer(uint8_t error_code, void* readAddress, uint8_t sizeOfData);
// void writeToMessageBuffer(uint8_t error_code, const char* str);
// void writeToMessageBuffer(uint8_t error_code, const __FlashStringHelper* fstr);
bool findCommand();
bool verifyCommandOperandLength(uint8_t expected_length);
bool verifyCommand();
bool loadCommand();
void sendResponsePacket();
bool shouldSendResponse();
void sendBadFunctionAddressError();
bool verifyValidCurveConfig(uint8_t start_index);
bool ensureActuatorIsFree(uint8_t actuator_index);

#endif /* PACKET_COMM_H */