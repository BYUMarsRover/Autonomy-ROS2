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


#endif
