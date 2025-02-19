#include "Arduino.h"
#include "Globals.h"
#include "Wheels.h"
using namespace std;

// Debug code is technically less performant so it may
// be desirable to disable it when unnecessary
#define DEBUG
#ifdef DEBUG
String debugMessage = "";
#endif

//Create object of type "Wheels" names "wheels"
Wheels wheels;

//Clicker state
unsigned long lastClickForward = 0;
unsigned long lastClickBackward = 0;
unsigned long lastClickDuration = 0;
bool clicking = false;

//Variables to store current yaw and pitch of FPV servo
float currentYaw = 375.0; //default position
float currentPitch = 300.0; //default position

//Heartbeat status
bool cardiacArrest = false;

//IR reader
unsigned int sensorArray[2];
unsigned long currentTime = 0;
unsigned long lastMessageTime = 0;

//Motor card reset queue
unsigned short motorStatuses[NUM_WHEELS];
unsigned long motorTimers[NUM_WHEELS];

// Buffer to hold the incoming NMEA sentence
char buffer[BUFFERSIZE];
int index = 0;
void readNMEAData() {
  // Read the data from Serial
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '$') { // Start of a new sentence
      clearBuffer();
      index = 0;
    } 
    else if (c == '*') {
      buffer[index] = '\0'; // End of the sentence
      parseNMEA(buffer); // Call function to parse the sentence
      break;
    } 
    else {
      if (index < BUFFERSIZE - 1) { // Ensure space for null terminator
        buffer[index] = c;
        index += 1;
      } 
      else {
        clearQueue();
        clearBuffer();

        #ifdef DEBUG
        debugMessage += "Arduino: Data input was too long and has been discarded. ";
        #endif

        break;
      }
    }
  }
}

void clearQueue() {
  while (Serial.available()) {
    char c = Serial.read();
  }
}

void clearBuffer() {
  memset(buffer, 0, sizeof(buffer));
}

// Function to parse/process the received NMEA sentence
void parseNMEA(char* sentence) {
  #ifdef DEBUG
    //debugMessage += "Arduino: Trying to read Orin data. ";
  #endif

  if (strncmp(sentence, "WHEEL", 5) == 0 && !cardiacArrest) {
    char* token = strtok(sentence + 6, ","); 
    for (int i = 0; i < NUM_WHEELS-1; i++) {
      int speed = atoi(token);
      token = strtok(NULL, ",");
      int dir = atoi(token);
      token = strtok(NULL, ",");
      wheels.wheelList[i].set_speed = speed;
      wheels.wheelList[i].dir = dir;
    }
    // Update the wheel parameters based on the received data
    wheels.writeParams();
  }
  else if (strncmp(sentence, "LASER", 5) == 0 && !cardiacArrest) {
    char* token = strtok(sentence + 6, ","); 
    int laser = atoi(token);
    handleLaser(laser);
  }
  else if (strncmp(sentence, "ELEVA", 5) == 0 && !cardiacArrest) {
    char* token = strtok(sentence + 6, ","); 
    int speed = atoi(token);
    token = strtok(NULL, ",");
    int dir = atoi(token);
    token = strtok(NULL, ",");
    handleElevator(speed, dir);
  }
  else if (strncmp(sentence, "CLICK", 5) == 0 && !cardiacArrest) {
    char* token = strtok(sentence + 6, ","); 
    int click = atoi(token);
    handleClickerCommand(click);
  }
  else if (strncmp(sentence, "FPVSV", 5) == 0 && !cardiacArrest) {
    char* token = strtok(sentence + 6, ","); 
    float yaw = atoi(token);
    token = strtok(NULL, ",");
    int pitch = atoi(token);
    handleFPV(yaw, pitch);
  }
  else if (strncmp(sentence, "HEART", 5) == 0) {
    char* token = strtok(sentence + 6, ","); 
    float elapsedConnection = atoi(token);
    handleHeartbeat(elapsedConnection);
  }
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

void handleElevator(int speed, int dir) {
  // If the top limit switch (active HIGH) is NOT pressed OR if the elevator is moving DOWN (0)
  // If the bottom limit switch (active HIGH) is NOT pressed OR if the elevator is moving UP (1)
  // Send the elevator commands as normal
  if ((!digitalRead(ELEVATOR_TOP_LIMIT_SWITCH) || dir == 0) 
   && (!digitalRead(ELEVATOR_BOTTOM_LIMIT_SWITCH) || dir == 1)) {
    wheels.wheelList[6].set_speed = speed;
    wheels.wheelList[6].dir = dir;

    #ifdef DEBUG
      debugMessage += "Arduino: No limit switch detected. Speed: " + String(speed);
      debugMessage += " | Dir: " + String(dir);
    #endif
  }
  // If the top limit switch (active HIGH) IS pressed AND the elevator is moving UP (1)
  // If the bottom limit swtich (active HIGH) is pressed AND the elevator is moving DOWN (0)
  else {
    wheels.wheelList[6].set_speed = STOP_WHEELS;
    wheels.wheelList[6].dir = STOP_WHEELS;

    #ifdef DEBUG
      debugMessage += "Arduino: We've been stopped by a limit switch! Speed: " + String(speed);
      debugMessage += " | Dir: " + String(dir);
    #endif
  }
}

void handleClickerCommand(bool click) {
  if (click == 1 && !clicking) {
    digitalWrite(CLICKER_DIR, HIGH);
    digitalWrite(CLICKER_ENABLE, HIGH);
    lastClickForward = millis();
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
}

void handleHeartbeat(float elapsedTime) {
  //Code to check if elapsed time between comms exceeds the value set in
  //MAXTIMEDIFF
  if (elapsedTime >= MAXTIMEDIFF) {
    //Stop all wheels and elevator
    for (int i = 0; i < NUM_WHEELS; i++) {
      wheels.wheelList[i].set_speed = STOP_WHEELS;
      wheels.wheelList[i].dir = STOP_WHEELS;
    }
    wheels.writeParams();

    //Turn off laser (may not want this?)
    digitalWrite(ARDUINO_LED, LOW);
    digitalWrite(LASER_CTRL, LOW);

    //Stop all wheel processing until heartbeat returns
    cardiacArrest = true;

    #ifdef DEBUG
      debugMessage += "Arduino: Our heart has stopped!";
    #endif
  }
  else if (cardiacArrest) {
    //cardiacArrest = false;
  }
}

void sendIRData() {
  // Construct NMEA sentence for IR data
  String sentence = "$IRLIG,";

  for (int i = 0; i < 2; i++) {
    sentence += String(sensorArray[i]) + ",";
  }

  sentence += "*"; // End sentence

  // Send data
  Serial.println(sentence);
  Serial.flush();
}

#ifdef DEBUG
void sendDebugData() {
  // Construct NMEA sentence for debug data
  String sentence = "$DEBUG,";

  sentence += debugMessage;

  sentence += "*"; // End sentence

  // Send data
  Serial.println(sentence);
  Serial.flush();

  // Clear debug queue
  debugMessage = "";
}
#endif

void handleClickerControl() {
  if (clicking && ((millis() > lastClickForward + MAX_CLICK_TIME) || (digitalRead(LIMIT_SWITCH) == HIGH))) {
      lastClickBackward = millis();
      lastClickDuration = lastClickBackward - lastClickForward;
      clicking = false;
      digitalWrite(CLICKER_ENABLE, HIGH);
      digitalWrite(CLICKER_DIR, LOW);
  }
  else if (!clicking && (millis() > lastClickBackward + min(lastClickDuration, MAX_CLICK_TIME))) {
      digitalWrite(CLICKER_ENABLE, LOW);
  }
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
        if(digitalRead(motorErrorPins[i])) {
          // A power cycle would be prefferable to
          // disable->enable, but the 2023 PCB does
          // not have power cycle capability
          #ifdef DEBUG
          debugMessage += "Arduino: Reseting motor #"+String(i);
          #endif
          digitalWrite(motorResetPins[i], LOW);
          motorStatuses[i] = RESETING;
          motorTimers[i] = millis() + MOTOR_RESET_TIME;
        }
        break;
      case RESETING:
        if(motorTimers[i] <= millis()) {
          // A power cycle would be prefferable to
          // disable->enable, but the 2023 PCB does
          // not have power cycle capability
          digitalWrite(motorResetPins[i], HIGH);
          motorStatuses[i] = STARTING;
          motorTimers[i] = millis() + MOTOR_STARTUP_TIME;
        }
        break;
      case STARTING:
        if(motorTimers[i] <= millis()) {
          motorStatuses[i] = RUNNING;
        }
        break;
      #ifdef DEBUG
      default:
        debugMessage += "Arduino: Motor " + String(i) + " in a untracked state: " + String(motorStatuses[i]);
      #endif
    }
  }
}

void setup() {
  // Configure the baud rate.
  Serial.begin(115200);
  #ifdef DEBUG
  debugMessage = "Arduino: Connection handshake.";
  sendDebugData();
  #endif

  //sets all pins as inputs or outputs
  setPinModes();

  //configure the pwm for the fpv servos
  configurePwm();

  //Write the default parameters contained in the Wheels class to the wheels
  for (int i = 0; i < NUM_WHEELS; i++) {
    wheels.wheelList[i].set_speed = STOP_WHEELS;
    wheels.wheelList[i].dir = STOP_WHEELS;
  }
  wheels.writeParams();

  //Write default values to hand actuators
  digitalWrite(ARDUINO_LED, LOW);
  digitalWrite(CLICKER_ENABLE, LOW);
  digitalWrite(CLICKER_DIR, LOW);
  digitalWrite(LASER_CTRL, LOW);

  //Prepare motor reset queue
  memset(motorStatuses, short(RUNNING), sizeof(motorStatuses));
  memset(motorTimers, MOTOR_STARTUP_TIME, sizeof(motorTimers));
}


void loop() {
  // Read and process NMEA data from serial
  readNMEAData();

  // Hardware control
  handleClickerControl();

  // This is just some defensive programming from chasing down a bug
  currentYaw = constrain(currentYaw, 125, 625);
  currentPitch = constrain(currentPitch, 135, 475);
  // Update the duty cycles for the fpv servos
  OCR1A = currentYaw;
  OCR1B = currentPitch;

  //Handle IR and motor resets at slower rate
  currentTime = millis();
  if( currentTime - lastMessageTime >= IRPERIOD ) {
    sensorArray[0] = analogRead(GRIP_IR1);
    sensorArray[1] = analogRead(GRIP_IR2);
    sendIRData();

    //Also check motors intermitently for speed
    //    handleMotorCardErrors();
    
    lastMessageTime = currentTime;
  }

  #ifdef DEBUG
  if (debugMessage != "")
    sendDebugData();
  #endif
}

void configurePwm() {
    // Configure Timer 1
    TCCR1A = 0; // Clear Control Register A
    TCCR1B = 0; // Clear Control Register B

    // Set Fast PWM mode with ICR1 as TOP
    TCCR1A |= (1 << WGM11);
    TCCR1B |= (1 << WGM13) | (1 << WGM12);

    // Set prescaler to 64
    TCCR1B |= (1 << CS11) | (1 << CS10);

    // Set ICR1 for 50 Hz
    ICR1 = 4999;

    // Set OCR1A and OCR1B duty cycle which is pins 11 and 12
    OCR1A = currentYaw;
    OCR1B = currentPitch;
    
    // Enable PWM output on pin 11 and 12 (OC1A and OC1B)
    TCCR1A |= (1 << COM1A1);
    TCCR1A |= (1 << COM1B1);
    
}

// This method sets the pins for each motor
void setPinModes() {
  //Wheels
  pinMode(RIGHT_FRONT_WHEEL_SET_SPEED, OUTPUT);
  pinMode(RIGHT_FRONT_WHEEL_DIR, OUTPUT);
  pinMode(RIGHT_FRONT_WHEEL_ENABLE, OUTPUT);
  pinMode(RIGHT_FRONT_WHEEL_ERROR, INPUT_PULLUP);

  pinMode(RIGHT_MIDDLE_WHEEL_SET_SPEED, OUTPUT);
  pinMode(RIGHT_MIDDLE_WHEEL_DIR, OUTPUT);
  pinMode(RIGHT_MIDDLE_WHEEL_ENABLE, OUTPUT);
  pinMode(RIGHT_MIDDLE_WHEEL_ERROR, INPUT_PULLUP);

  pinMode(RIGHT_REAR_WHEEL_SET_SPEED, OUTPUT);
  pinMode(RIGHT_REAR_WHEEL_DIR, OUTPUT);
  pinMode(RIGHT_REAR_WHEEL_ENABLE, OUTPUT);
  pinMode(RIGHT_REAR_WHEEL_ERROR, INPUT_PULLUP);

  pinMode(LEFT_FRONT_WHEEL_SET_SPEED, OUTPUT);
  pinMode(LEFT_FRONT_WHEEL_DIR, OUTPUT);
  pinMode(LEFT_FRONT_WHEEL_ENABLE, OUTPUT);
  pinMode(LEFT_FRONT_WHEEL_ERROR, INPUT_PULLUP);

  pinMode(LEFT_MIDDLE_WHEEL_SET_SPEED, OUTPUT);
  pinMode(LEFT_MIDDLE_WHEEL_DIR, OUTPUT);
  pinMode(LEFT_MIDDLE_WHEEL_ENABLE, OUTPUT);
  pinMode(LEFT_MIDDLE_WHEEL_ERROR, INPUT_PULLUP);

  pinMode(LEFT_REAR_WHEEL_SET_SPEED, OUTPUT);
  pinMode(LEFT_REAR_WHEEL_DIR, OUTPUT);
  pinMode(LEFT_REAR_WHEEL_ENABLE, OUTPUT);
  pinMode(LEFT_REAR_WHEEL_ERROR, INPUT_PULLUP);

  //Elevator
  pinMode(ELEVATOR_SET_SPEED, OUTPUT);
  pinMode(ELEVATOR_DIR, OUTPUT);
  pinMode(ELEVATOR_ENABLE, OUTPUT);
  pinMode(ELEVATOR_ERROR, INPUT_PULLUP);
  pinMode(ELEVATOR_TOP_LIMIT_SWITCH, INPUT_PULLUP);
  pinMode(ELEVATOR_BOTTOM_LIMIT_SWITCH, INPUT_PULLUP);

  //IR sensors
  pinMode(GRIP_IR1, INPUT);
  pinMode(GRIP_IR2, INPUT);

  //Hand actuators
  pinMode(LIMIT_SWITCH, INPUT);
  pinMode(CLICKER_DIR, OUTPUT);
  pinMode(CLICKER_ENABLE, OUTPUT);
  pinMode(LASER_CTRL, OUTPUT);
  pinMode(ARDUINO_LED, OUTPUT);

  // FPV Camera servos
  pinMode(FPV_YAW, OUTPUT);
  pinMode(FPV_PITCH, OUTPUT);
}
