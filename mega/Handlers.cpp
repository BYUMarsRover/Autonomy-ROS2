#include "Handlers.h"

void handleDrive(int speeds[], int dirs[]) {
  for (int i = 0; i < NUM_WHEELS-1; i++) {
    wheels.wheelList[i].set_speed = speeds[i];
    wheels.wheelList[i].dir = dirs[i];
  }
  
  // Update the wheel parameters based on the received data
  wheels.writeParams();
}

void handleLaser(int laser) {
  if (laser == 1) {
    digitalWrite(ARDUINO_LED, HIGH);
    digitalWrite(LASER_CTRL, HIGH);
  }
  else{
    digitalWrite(ARDUINO_LED, LOW);
    digitalWrite(LASER_CTRL, LOW);
  }
}

void handleElevator(int speed, int dir, bool force) {
  // Always move the elevator when forced
  if (force) {
    wheels.wheelList[6].set_speed = speed;
    wheels.wheelList[6].dir = dir;

    #if DEBUG
    if (speed != 0) {
      prepareDebugData("Arduino: Overriding limit switch checks. Speed: ");
      prepareDebugData(speed);
      prepareDebugData(" | Dir: ");
      prepareDebugData(dir);
      prepareDebugData(" (1 is up)");
    }
    #endif
  }

  // If the top limit switch (active HIGH) is NOT pressed OR if the elevator is moving DOWN (0)
  // If the bottom limit switch (active HIGH) is NOT pressed OR if the elevator is moving UP (1)
  // Send the elevator commands as normal
  else if ((!digitalRead(ELEVATOR_TOP_LIMIT_SWITCH) || dir == 0) 
        && (!digitalRead(ELEVATOR_BOTTOM_LIMIT_SWITCH) || dir == 1)) {
    wheels.wheelList[6].set_speed = speed;
    wheels.wheelList[6].dir = dir;

    #if DEBUG
    if (speed != 0) {
      prepareDebugData("Arduino: No limit switch detected. Speed: ");
      prepareDebugData(speed);
      prepareDebugData(" | Dir: ");
      prepareDebugData(dir);
      prepareDebugData(" (1 is up)");
    }
    #endif
  }
  // If the top limit switch (active HIGH) IS pressed AND the elevator is moving UP (1)
  // If the bottom limit swtich (active HIGH) is pressed AND the elevator is moving DOWN (0)
  else {
    wheels.wheelList[6].set_speed = STOP_WHEELS;
    wheels.wheelList[6].dir = STOP_WHEELS;

    #if DEBUG
      prepareDebugData("Arduino: We've hit a limit switch! Speed: ");
      prepareDebugData(speed);
      prepareDebugData(" | Dir: ");
      prepareDebugData(dir);
      prepareDebugData(" (1 is up)");
    #endif
  }

  // Update the wheel parameters based on the received data
  wheels.writeParams();
}

void handleClickerCommand(bool click) {
  if (click == 1 && !clicking) {
    digitalWrite(CLICKER_DIR, HIGH);
    digitalWrite(CLICKER_ENABLE, HIGH);
    lastClickForward = currentTime;
    clicking = true;
  }
}

void handleFPV(float yawCmd, float pitchCmd) {
  // Updates the current duty cycle and ensures servo limits for yaw
  if (currentYaw + yawCmd > MAX_PWM_DUTY_CYCLE_YAW) {
    currentYaw = MAX_PWM_DUTY_CYCLE_YAW;
  } 
  else if (currentYaw + yawCmd < MIN_PWM_DUTY_CYCLE_YAW) {
    currentYaw = MIN_PWM_DUTY_CYCLE_YAW;
  } else {
    currentYaw += yawCmd;
  }
  // Updates the current duty cycle and ensures servo limits for pitch  
  if (currentPitch - pitchCmd > MAX_PWM_DUTY_CYCLE_PITCH) {
    currentPitch = MAX_PWM_DUTY_CYCLE_PITCH;
  } 
  else if (currentPitch - pitchCmd < MIN_PWM_DUTY_CYCLE_PITCH) {
    currentPitch = MIN_PWM_DUTY_CYCLE_PITCH;
  } else {
    currentPitch -= pitchCmd;
  }
  
  // This is just some defensive programming from chasing down a bug
  currentYaw = constrain(currentYaw, 125, 625);
  currentPitch = constrain(currentPitch, 135, 475);
  // Update the duty cycles for the fpv servos
  OCR1A = currentYaw;
  OCR1B = currentPitch;
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
    #if DEBUG
      prepareDebugData("Arduino: Our heart is back -- thank goodness!");
    #endif
    
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

void handleClickerControl() {
  if (clicking && ((currentTime > lastClickForward + MAX_CLICK_TIME) || (digitalRead(LIMIT_SWITCH) == HIGH))) {
      lastClickBackward = currentTime;
      lastClickDuration = lastClickBackward - lastClickForward;
      clicking = false;
      digitalWrite(CLICKER_ENABLE, HIGH);
      digitalWrite(CLICKER_DIR, LOW);
  }
  else if (!clicking && (currentTime > lastClickBackward + min(lastClickDuration, MAX_CLICK_TIME))) {
      digitalWrite(CLICKER_ENABLE, LOW);
  }
}

void handleMotorCardErrors() {
  //Resets motors to avoid error lock
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
        if(digitalRead(wheels.wheelList[i].error_pin)) {
          // A power cycle would be preferable to
          // disable->enable, but the 2023 PCB does
          // not have power cycle capability
          #if DEBUG
          prepareDebugData("Arduino: Reseting motor #");
          prepareDebugData(i);
          #endif
          digitalWrite(wheels.wheelList[i].enable_pin, LOW);
          motorStatuses[i] = RESETING;
          motorTimers[i] = currentTime + MOTOR_RESET_TIME;
        }
        break;
      case RESETING:
        if(motorTimers[i] <= currentTime) {
          // A power cycle would be preferable to
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
      #if DEBUG
      default:
        prepareDebugData("Arduino: Motor ");
        prepareDebugData(i);
        prepareDebugData(" in a untracked state: ");
        prepareDebugData(motorStatuses[i]);
      #endif
    }
  }
}
