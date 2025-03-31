#ifndef CIRCQUE_H
#define CIRCQUE_H

#include <Arduino.h>
#include "Globals.h"
#include "Communication.h"

void queuePrint(const char* msg);
void dequeuePrint(char* output, int maxLength);

#endif
