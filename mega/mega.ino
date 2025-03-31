#include "Globals.h"
#include "Wheels.h"
#include "Setup.h"
#include "Communication.h"
#include "Handlers.h"
using namespace std;

void setup() {
  setupArduino();
}

void loop() {
  // Update time
  currentTime = millis();

  // Handle serial communication
  readNMEAData();
  writeNMEAData();

  // Hardware control
  handleClickerControl();

  // Check for disconnect from Orin
  if( currentTime - lastContactTime >= MAXTIMEDIFF ) {
    killEverything();

    // Blink Arduino LED if disconnected
    if (currentTime - prevLEDBlink >= blinkInterval) {
      prevLEDBlink = currentTime;

      // If LED is on, turn it off; if off, turn it on
      if (digitalRead(ARDUINO_LED) == HIGH) {
        digitalWrite(ARDUINO_LED, LOW);
      } else {
        digitalWrite(ARDUINO_LED, HIGH);
        falseLED = true;
      }

      //Try to open communications again
      // This code is inside the blink condition to make it 
      // trigger less often, in order to spare I/O capacity
      if (currentTime - lastContactTime >= 5*MAXTIMEDIFF) {
        openSerial();
      }
    }
  }
  else if(falseLED) { // Makes sure LED is low after reconnection (otherwise 50/50)
    digitalWrite(ARDUINO_LED, LOW);
    falseLED = false;
  }

  // Handle IR, motor resets, and limit switches at slower rate
  if( currentTime - lastSlowTime >= SLOWPERIOD ) {
    sensorArray[0] = analogRead(GRIP_IR1);
    sensorArray[1] = analogRead(GRIP_IR2);
    sendIRData();

    //Also check motors intermitently for speed
    handleMotorCardErrors();

    // Check for limit switch actuation outside of commands
    //  Limit switches are also checked at the time of commands,
    //  but this makes the rover more robust to lost commms
    if (((digitalRead(ELEVATOR_TOP_LIMIT_SWITCH)    && wheels.wheelList[6].dir == 1)
      || (digitalRead(ELEVATOR_BOTTOM_LIMIT_SWITCH) && wheels.wheelList[6].dir == 0))
      && wheels.wheelList[6].set_speed > STOP_WHEELS) {
      #if DEBUG
        prepareDebugData("Arduino: We've hit a limit switch between commands!");
        prepareDebugData(" Dir: ");
        prepareDebugData(wheels.wheelList[6].dir);
        prepareDebugData(" (1 is up)");
      #endif

      wheels.wheelList[6].set_speed = STOP_WHEELS;
      wheels.wheelList[6].dir = STOP_WHEELS;
    }
    
    //Also send debug messages intermitently
    // This code is inside the IR condition to make it 
    // trigger less often, in order to spare I/O capacity
    #if DEBUG
    if (debugMessage[0] != '\0')
      sendDebugData();
    #endif
    
    lastSlowTime = currentTime;
  }
}
