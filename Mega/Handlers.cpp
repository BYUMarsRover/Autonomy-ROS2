#include "Handlers.h"

void handleDrive(int speeds[], int dirs[]) {
  for (int i = 0; i < NUM_WHEELS-1; i++) {
    wheels.wheelList[i].set_speed = speeds[i];
    wheels.wheelList[i].dir = dirs[i];
  }
  
  // Update the wheel parameters based on the received data
  wheels.writeParams();
}


void handleElevator(int speed, int dir) {
  // If the top limit switch (active HIGH) is NOT pressed OR if the elevator is moving DOWN (0)
  // If the bottom limit switch (active HIGH) is NOT pressed OR if the elevator is moving UP (1)
  // Send the elevator commands as normal
  if ((!digitalRead(ELEVATOR_TOP_LIMIT_SWITCH) || dir == 0) 
   && (!digitalRead(ELEVATOR_BOTTOM_LIMIT_SWITCH) || dir == 1)) {
    wheels.wheelList[6].set_speed = speed;
    wheels.wheelList[6].dir = dir;

  }
  // If the top limit switch (active HIGH) IS pressed AND the elevator is moving UP (1)
  // If the bottom limit swtich (active HIGH) is pressed AND the elevator is moving DOWN (0)
  else {
    wheels.wheelList[6].set_speed = STOP_WHEELS;
    wheels.wheelList[6].dir = STOP_WHEELS;

  }

  // Update the wheel parameters based on the received data
  wheels.writeParams();
}


void handleHeartbeat(float elapsedTime) {
  //Code to check if elapsed time between comms exceeds the value set in
  //MAXTIMEDIFF
  if (elapsedTime >= MAXTIMEDIFF) {
    killEverything();
    digitalWrite(ARDUINO_LED, LOW); //ARDUINO_LED is hijacked here to differentiate between cardiac arrest and orin disconnect

    //Stop all wheel processing until heartbeat returns
    cardiacArrest = true;

    #if DEBUG
      prepareDebugData("Arduino: Our heart has stopped!");
    #endif
  }
  else if (cardiacArrest) {
    cardiacArrest = false;
  }
}

void killEverything() {
    //Stop all wheels and elevator
    for (int i = 0; i < NUM_WHEELS; i++) {
      wheels.wheelList[i].set_speed = STOP_WHEELS;
      wheels.wheelList[i].dir = STOP_WHEELS;
    }
    wheels.writeParams();
  
    //Turn off laser (may not want this?)
    digitalWrite(LASER_CTRL, LOW);
  }

void handleMotorCardErrors() {
  // Resets motors to avoid error lock
  // We (the 2024 team) believe it is preferrable
  // to try reseting the motor card and ignore any
  // errors during competition if it means the motors
  // can run when we need them. Best practice suggests
  // that we find the source of motor card errors
  // to eliminate the problem at its source, but this
  // still improves reliablity (at the potential cost
  // of longevity).
  for(int i = 0; i < NUM_WHEELS; i++) {
    switch(motorStatuses[i]) {
      case RUNNING:
        if(!digitalRead(wheels.wheelList[i].error_pin)) {
          // A power cycle would be prefferable to
          // disable->enable, but the 2023 PCB does
          // not have power cycle capability
          digitalWrite(wheels.wheelList[i].enable_pin, LOW);
          motorStatuses[i] = RESETING;
          motorTimers[i] = currentTime + MOTOR_RESET_TIME;
        }
        break;
      case RESETING:
        if(motorTimers[i] <= currentTime) {
          // A power cycle would be prefferable to
          // disable->enable, but the 2023 PCB does
          // not have power cycle capability
          digitalWrite(wheels.wheelList[i].enable_pin, HIGH);
          motorStatuses[i] = STARTING;
          motorTimers[i] = currentTime + MOTOR_STARTUP_TIME;
        }
        break;
      case STARTING:
        if(motorTimers[i] <= currentTime) {
          motorStatuses[i] = RUNNING;
        }
        break;
    }
  }
}
