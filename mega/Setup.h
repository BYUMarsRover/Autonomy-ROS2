#ifndef SETUP_H
#define SETUP_H

#include <Arduino.h>
#include "Globals.h"
#include "Wheels.h"
#include "Communication.h"

void setupArduino();
void openSerial();
void configurePwm();
void setPinModes();

#endif
