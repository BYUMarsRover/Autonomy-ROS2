#ifndef WHEEL_H
#define WHEEL_H

#include <Arduino.h>

class Wheel {
public:

  Wheel() {
    //these are default parameters for writing on bootup 
    this->set_speed_pin = 0;
    this->enable_pin = 0;
    this->dir_pin = 0;
    this->actual_speed_pin = 0;
    this->error_pin = 0;   
    this->set_speed = 0;
    this->actual_speed = 0;
    this->dir = true;
    this->error = false;
    this->is_right_side_wheel = true;
  }

//is used by Wheels header file. 
  void init(uint8_t set_speed_pin, uint8_t enable_pin, uint8_t dir_pin, uint8_t actual_speed_pin, uint8_t error_pin, bool is_right_side_wheel){
    this->set_speed_pin = set_speed_pin;
    this->enable_pin = enable_pin;
    this->dir_pin = dir_pin;
    this->actual_speed_pin = actual_speed_pin;
    this->error_pin = error_pin;
    this->is_right_side_wheel = !is_right_side_wheel;
    
    digitalWrite(enable_pin, true);
  }
  
  ~Wheel(){}

  void writeParams() {
    /*
    * The Escon Drive Controllers require a minuimum-maximum range for the duty cycle (pwm) of 10% - 90%. So the desired speed is mapped from
    *  a range of [0,255] to [255*.1,255*.9].
    */
    analogWrite(set_speed_pin, map(set_speed,0,255,(255 * .1),(255 * .9)));
    //WILL NEED TO READJUST THIS
    digitalWrite(dir_pin, is_right_side_wheel == dir);
  }

  uint8_t set_speed_pin;
  uint8_t dir_pin;
  uint8_t enable_pin;
  uint8_t actual_speed_pin;
  uint8_t error_pin;  
  byte set_speed;
  byte actual_speed; 
  bool dir;
  bool error;
  bool is_right_side_wheel;
};


#endif
