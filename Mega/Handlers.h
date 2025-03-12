#ifndef HANDLERS_H
#define HANDLERS_H

#include "Globals.h"
#include "Wheels.h"
#include "Communication.h"

void handleDrive(int speeds[], int dirs[]);
void handleElevator(int speed, int dir);
void handleHeartbeat(float elapsedTime);
void handleMotorCardErrors(); // But we live with it
void killEverything();

#endif
