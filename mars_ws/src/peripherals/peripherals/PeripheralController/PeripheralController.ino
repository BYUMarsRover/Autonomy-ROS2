#ifndef ARM_ARDUINO_INO
#define ARM_ARDUINO_INO

//#include <ros.h>
//#include <rover_msgs/bat_stat_grip_elev_arduino.h>
#define FASTLED_INTERNAL
#include <FastLED.h>

// Elevator and Gripper
#define FLT 2 // D2 (fault)
#define INA 3 // D3
#define INB 4 // D4
#define PWM 5 // D5

// Battery
#define BATTERY_MONITOR_INPUT_PIN A0  // A0
#define BATTERY_MONITOR_PREAMBLE  0x2
//#define BATTERY_MONITOR_BATTERY_ADC_MIN 729

// Indicator
#define STATUS_INDICATOR_LED_PIN      8 // D8
#define STATUS_INDICATOR_NUM_LEDS     32
#define STATUS_INDICATOR_BRIGHTNESS   25
#define STATUS_INDICATOR_PREAMBLE     0x1
#define STATUS_INDICATOR_COUNTER_MAX  250

unsigned long previousTime = 0;
int battCounter = 0;
int statCounter = 0;
bool engage = false;
bool timerActive = false;

// Define the array of leds
CRGB leds[STATUS_INDICATOR_NUM_LEDS];
enum led_mode_states {AUTONOMOUS, TELEOPERATION, ARRIVAL, IDLE};
led_mode_states led_mode;

void setArrayColor(char red, char green, char blue)
{
  for (int x=0;x<STATUS_INDICATOR_NUM_LEDS;x++)
  {
    leds[x] = CRGB(red,green,blue);
  }
  FastLED.show();
}

void setup()
{
  // Arduino_rover_status setup
  Serial.begin(9600);


  // Gripper pin setup //
  // FAULT - LOW: Overcurrent condition or thermal shutdown
  //         HIGH: Normal Operation
  pinMode(FLT, INPUT);

  // ENABLE - LOW: Enable driver outputs
  //          HIGH: Enable Tri-State ??????????????
  pinMode(INA, OUTPUT);

  // DIRECTION - LOW: Reverse
  //             HIGH: Forward
  pinMode(INB, OUTPUT);

  // PULSE WIDTH MODULATION - 0: 0% Duty Cycle
  //                          255: 100% Duty Cycle
  pinMode(PWM, OUTPUT);

  // Initialize Gripper pins //
  // Sets ENABLE to enable driver outputs
  digitalWrite(INA, LOW);

  // Sets DIRECTION to forward
  digitalWrite(INB, HIGH);

  // Sets PULSE WIDTH MODULATION to 0% duty cycle
  analogWrite(PWM, 0);
  

  // Battery Setup //
  pinMode(BATTERY_MONITOR_INPUT_PIN, INPUT);


  // Inidicator LED setup //
  FastLED.addLeds<WS2812B, STATUS_INDICATOR_LED_PIN, GRB>(leds, STATUS_INDICATOR_NUM_LEDS);
  FastLED.setBrightness(STATUS_INDICATOR_BRIGHTNESS);
  setArrayColor(255, 255, 255);    // Flash white for ok signal
  led_mode = IDLE;


  Serial.flush();
}

void statusIndicatorTick()
{
  if (led_mode == AUTONOMOUS) {
    setArrayColor(255,0,0);
    timerActive = false;
  }
  else if (led_mode == ARRIVAL)
  {
    if (timerActive == false){
        previousTime = millis();
        timerActive = true;
        setArrayColor(0,255,0);
    }
    if (timerActive && millis() - previousTime >= 2000 && millis() - previousTime <=4000) {
        Serial.println("5 seconds passed");
        setArrayColor(0,0,0);
    }
    else{
      timerActive = false;
    }
  }
  else if (led_mode == IDLE){
    timerActive = false;
    setArrayColor(0,0,0);
  }
  else if (led_mode == TELEOPERATION) {
    setArrayColor(0,0,255);
    timerActive = false;
  }
  else {
    setArrayColor(0,0,0);
    // timerActive = false;
  }
}
// previousTime = millis(); // Reset timer safely

void batteryTick()
{
  if (battCounter++ < 1000) return;
  battCounter = 0;
  int reading = analogRead(BATTERY_MONITOR_INPUT_PIN);

  // You can't write to serial correctly without flushing the buffer
  Serial.flush();

  Serial.println(reading);
  Serial.flush();
}
  
char data_array[128];
int stored_data_index = 0;

void decoder()
{
  /////////////////////
  // BUFFER EXTRACTION/READING

  // reply only when you receive data:
  if (Serial.available() > 0) {\
    // setup for loop where we put serial buffer bytes into local RAM buffer
    int how_many_bytes = Serial.available();
    bool has_message = false;
    int end_of_message_index = -1;
    
    // each iteration of the loop gets one character
    for (int i = 0; i < how_many_bytes; i++) {
      char read_char = Serial.read();
      // FOR DEBUGGING ONLY, PRINT STATEMENTS ARE INTERPRETED AS BATTERY VOLTAGE READINGS
//      Serial.println("read char");
//      Serial.println(read_char);
      // we defined an interface where a semicolon represents the end of the message, so we look for semicolons
      data_array[stored_data_index] = read_char;
      stored_data_index++;
      
      if (read_char == ';') {
        has_message = true;
        // we keep track of the end of message,
        end_of_message_index = stored_data_index - 1;
        
        // we break to make sure that we don't store two valid commands. And this helps us to make parsing easier and prevents some edge cases that mess us up 
        // (if you understand buffers, then you can change this, otherwise, don't)
        break;
      }
    }

    /////////////////////////
    // PARSING
    //if we have a message, then we continue with parsing, otherwise, this function is done
    if (has_message) {
      if(data_array[0] == 'G') {
        // this parses the gripper_elev commands
        // first, we find the index of the colon delimiter
        char *delimiter_index = data_array;
        while(*delimiter_index != ':') {
          delimiter_index++;
        }
        // we found the index of the colon delimiter
        *delimiter_index = '\0';
        // now the delimiter points to the start of the elevator field
        delimiter_index++;
        
        data_array[end_of_message_index] = '\0';
        char *gripper = data_array;
        //now gripper will point to the start of the integer field (ignoring the 'G' character)
        gripper++;
        int gripper_int = atoi(gripper);
        int elevator_int = atoi(delimiter_index);
        // FOR DEBUGGING ONLY; PRINT STATEMENTS ARE INTERPRETED AS BATTERY VOLTAGE READINGS
//         Serial.println(gripper_int);
//         Serial.println(elevator_int);

        ///////////////////
        // HAVING PARSED, WE NOW DO THE FUNCITONALITY
        if (gripper_int == 0) {
          analogWrite(PWM, 0);
        } else if (gripper_int > 0) {
      
          digitalWrite(INA, HIGH);
          digitalWrite(INB, LOW);
          analogWrite(PWM, gripper_int);
        } else if (gripper_int < 0) {
      
          digitalWrite(INA, LOW);
          digitalWrite(INB, HIGH);
          analogWrite(PWM, -255 - gripper_int);
        }
      } else if(data_array[0] == 'L') {
        ////////////////////
        // PARSING LED COMMAND
        
        data_array[end_of_message_index] = '\0';
        // FOR DEBUGGING ONLY; PRINT STATEMENTS ARE INTERPRETED AS BATTERY VOLTAGE READINGS
//        Serial.println("data array:");
//        Serial.println(data_array);
        char* nav_field = data_array;
        nav_field++;
        int navigation_state = atoi(nav_field);

        ///////////////////
        // HAVING PARSED, THE FOLLOWING IS FUNCTIONALITY
        if (navigation_state != -1) {
          led_mode = static_cast<led_mode_states>(navigation_state);
        }
        // FOR DEBUGGING ONLY; PRINT STATEMENTS ARE INTERPRETED AS BATTERY VOLTAGE READINGS
//        Serial.println("navigation state:");
//        Serial.println(navigation_state);
      } else {
        // you done messed up A-Aron, we erase the buffer
        // FOR DEBUGGING ONLY; PRINT STATEMENTS ARE INTERPRETED AS BATTERY VOLTAGE READINGS
        // println("invalid command detected");
        stored_data_index = 0;
      } 
      // we have parsed our message, now we need to reset the buffer
      stored_data_index = 0;
    }
  }
}

void loop()
{
  // Serial messages
  decoder();
  // Indicator LED
  unsigned long lastTime = 0;
  statusIndicatorTick();
  unsigned long currentTime = millis();
  Serial.println(currentTime - lastTime);  // Print time per loop iteration
  lastTime = currentTime;
  // Battery 
  batteryTick();
}

#endif 
