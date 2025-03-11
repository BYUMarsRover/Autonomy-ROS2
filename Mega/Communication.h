#ifndef COMMS_H
#define COMMS_H

#include "Communication.h"
#include <Arduino.h>
#include "Globals.h"
#include "Handlers.h"

void readNMEAData();
void clearQueue();
void clearBuffer();
void parseNMEA(char* sentence);
void sendIRData();

#if DEBUG
void sendDebugData();
void prepareDebugData(int newMessageAsInt);
void prepareDebugData(const char* newMessage);
#endif

#endif