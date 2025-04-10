#ifndef COMMS_H
#define COMMS_H

#include <Arduino.h>
#include "Globals.h"
#include "Handlers.h"
#include "CircularQueue.h"

void readNMEAData();
void clearQueue();
void clearBuffer();
void parseNMEA(char* sentence);
void sendIRData();
void writeNMEAData();

#if DEBUG
void sendDebugData();
void prepareDebugData(int newMessageAsInt);
void prepareDebugData(const char* newMessage);
#endif

#endif
