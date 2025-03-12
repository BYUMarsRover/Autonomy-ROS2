#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>

// Debug code is technically less performant so it may
// be desirable to disable it when unnecessary
//#define DEBUG true

//Constant variable that sets the maximum delay between motor commands
//If this value is exceeded, it implies that communication has been lost between
//the Orin and the Mega and the Mega will shut down all motion in the wheels
const unsigned long MAXTIMEDIFF = 500; //ms
const unsigned int BUFFERSIZE = 256;
const unsigned int IRPERIOD = 100; //Period , not frequency (50 -> 20Hz)

//Number of motors on arduino  
const int NUM_WHEELS = 7;

//Wheel + Elevator pin defintions
const int ELEVATOR_SET_SPEED = 9;
const int ELEVATOR_ENABLE = 30;
const int ELEVATOR_DIR = 31;
const int ELEVATOR_ERROR = 32; 
const int ELEVATOR_ACTUAL_SPEED = A5; 

const int LEFT_FRONT_WHEEL_SET_SPEED = 45;
const int LEFT_FRONT_WHEEL_ENABLE = 49;
const int LEFT_FRONT_WHEEL_DIR = 53;
const int LEFT_FRONT_WHEEL_ERROR = 51;
const int LEFT_FRONT_WHEEL_ACTUAL_SPEED = A15;

const int LEFT_MIDDLE_WHEEL_SET_SPEED = 46;
const int LEFT_MIDDLE_WHEEL_ENABLE = 48;
const int LEFT_MIDDLE_WHEEL_DIR = 50;
const int LEFT_MIDDLE_WHEEL_ERROR = 52;
const int LEFT_MIDDLE_WHEEL_ACTUAL_SPEED = A9;

const int LEFT_REAR_WHEEL_SET_SPEED = 44;
const int LEFT_REAR_WHEEL_ENABLE = 38;
const int LEFT_REAR_WHEEL_DIR = 40;
const int LEFT_REAR_WHEEL_ERROR = 42;
const int LEFT_REAR_WHEEL_ACTUAL_SPEED = A3;

const int RIGHT_REAR_WHEEL_SET_SPEED = 2;
const int RIGHT_REAR_WHEEL_ENABLE = 27;
const int RIGHT_REAR_WHEEL_DIR = 25;
const int RIGHT_REAR_WHEEL_ERROR = 23;
const int RIGHT_REAR_WHEEL_ACTUAL_SPEED = A6;

const int RIGHT_MIDDLE_WHEEL_SET_SPEED = 3;
const int RIGHT_MIDDLE_WHEEL_ENABLE = 24;
const int RIGHT_MIDDLE_WHEEL_DIR = 26;
const int RIGHT_MIDDLE_WHEEL_ERROR = 28;
const int RIGHT_MIDDLE_WHEEL_ACTUAL_SPEED = A12;

const int RIGHT_FRONT_WHEEL_SET_SPEED = 4;
const int RIGHT_FRONT_WHEEL_ENABLE = 5;
const int RIGHT_FRONT_WHEEL_DIR = 6;
const int RIGHT_FRONT_WHEEL_ERROR = 7;
const int RIGHT_FRONT_WHEEL_ACTUAL_SPEED = A10;

//Constant variable used to shut down the wheels
const byte STOP_WHEELS = byte(0);

//Gripper IR pin definitions
const int GRIP_IR1 = A0;
const int GRIP_IR2 = A1;

//Hand Actuator pin definitions
const int CLICKER_DIR = 34;
const int CLICKER_ENABLE = 36;
const int LIMIT_SWITCH = 47;
const int LASER_CTRL = 22;
const int ARDUINO_LED = 13;

//Elevator
const int ELEVATOR_TOP_LIMIT_SWITCH = 43;
const int ELEVATOR_BOTTOM_LIMIT_SWITCH = 41;

//Clicker 
const int MAX_CLICK_TIME = 2100;

// FPV SERVO PIN DEFINITIONS
const int FPV_YAW = 11;
const int FPV_PITCH = 12;
const int MAX_PWM_DUTY_CYCLE_YAW = 625;
const int MIN_PWM_DUTY_CYCLE_YAW = 125;
const int MAX_PWM_DUTY_CYCLE_PITCH = 475;
const int MIN_PWM_DUTY_CYCLE_PITCH = 135;

//Motor reset
#define RUNNING 0
#define RESETING 1
#define STARTING 2
const unsigned int MOTOR_RESET_TIME = 2000; //[ms]
const unsigned int MOTOR_STARTUP_TIME = 10000; //[ms]

//////////////////////////////////
//   Declare Global variables   //
//////////////////////////////////
//This is probably not best programming practice for modification/debugging, but hopefully helps keep the code readable and easy to reorganize

//Debug
#if DEBUG
#define DEBUG_MSG_SIZE 256  // Size of the debug message buffer
extern char debugMessage[DEBUG_MSG_SIZE];  // Fixed-size buffer for the debug message
#endif

//Clicker state
extern unsigned long lastClickForward;
extern unsigned long lastClickBackward;
extern unsigned long lastClickDuration;
extern bool clicking;

//Variables to store current yaw and pitch of FPV servo
extern float currentYaw;
extern float currentPitch;

//Heartbeat status
extern bool cardiacArrest;
extern unsigned long lastContactTime;
extern unsigned long prevLEDBlink;
extern unsigned long blinkInterval;

//IR reader
extern unsigned int sensorArray[2];
extern unsigned long currentTime;
extern unsigned long lastMessageTime;

//Motor card reset queue
extern unsigned short motorStatuses[NUM_WHEELS];
extern unsigned long motorTimers[NUM_WHEELS];
#endif
