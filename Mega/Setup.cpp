#include "Setup.h"

// Initialize global variables
///////////////////////////////////////////////////////////////////////
#if DEBUG
char debugMessage[DEBUG_MSG_SIZE] = {0};
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
unsigned long lastContactTime = 0;
unsigned long prevLEDBlink = 0;
unsigned long blinkInterval = 500;  // 200ms -> 5Hz

//IR reader
unsigned int sensorArray[2] = {0, 0};
unsigned long currentTime = 0;
unsigned long lastMessageTime = 0;

//Motor card reset queue
unsigned short motorStatuses[NUM_WHEELS];
unsigned long motorTimers[NUM_WHEELS];
////////////////////////////////////////////////////////////////////

void setupArduino() {
  //Connect
  openSerial();

  //Sets all pins as inputs or outputs
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
  for (int i = 0; i < NUM_WHEELS; i++) {
    motorStatuses[i] = RUNNING;
    motorTimers[i] = MOTOR_STARTUP_TIME;
  }
}

void openSerial() {
  // Close any existing serial ports
  if (Serial)
    Serial.end();
    delay(0.5);
    
  // Configure the serial connection
  Serial.begin(115200);
  clearQueue();
  clearBuffer();
  while (!Serial) {}; // Give time to open

  // Establish handshake
  if (Serial.availableForWrite() > 0)
    Serial.println("$HANDS,*");
  #if DEBUG
  prepareDebugData("Arduino: Connection handshake confirmation.");
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
