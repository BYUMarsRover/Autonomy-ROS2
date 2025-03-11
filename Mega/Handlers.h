#ifndef HANDLERS_H
#define HANDLERS_H

#include "Globals.h"
#include "Wheels.h"
#include "Communication.h"

void handleDrive(int speeds[], int dirs[]);
void handleLaser(int laser);
void handleElevator(int speed, int dir);
void handleClickerCommand(bool click);
void handleFPV(float yawCmd, float pitchCmd);
void handleHeartbeat(float elapsedTime);
void handleClickerControl(); // This is not a "true" handler in that it only touches global variables and could simply run in loop
void handleMotorCardErrors(); // But we live with it
void killEverything();

#endif