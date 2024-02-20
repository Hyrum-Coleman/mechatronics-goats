#include "L298NMotorDriverMega.h"

// There are no default pins for this library
// ENX should use:
//    pins 11 and 12 for timer 1
//    pins 5 and 2 for timer 3
//    pins 6 and 7 for timer 4
// or pins 45 and 46 for timer 5

// INX can use any digital pin (doesn't have to be PWM).

const int ENA = 5; 
const int IN1 = 34;
const int IN2 = 32;
const int ENB = 6;
const int IN3 = 33;
const int IN4 = 35;

L298NMotorDriverMega md(ENA,IN1,IN2,ENB,IN3,IN4); // create object for motor driver

float f = 0.25;             //frequency in Hz
unsigned long t = 0;        //current time

void setup() {
  // put your setup code here, to run once:
  
  md.init();
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  double t = micros() / 1000000.0; //current time
  int M = 400 * sin(2 * PI * f * t); //Sinusoid motor voltage command
  // Turn on Motor 1 for 4 seconds /////////////////////////////////////////
  if (0.0 < t && t < 4.0) {
    md.setM1Speed(M);
  }
  else {
    md.setM1Brake(0);
  }
  // Turn on Motor 2 for 4 seconds /////////////////////////////////////////
  if (4.0 < t && t < 8.0) {
    md.setM2Speed(M);
  }
  else {
    md.setM2Brake(0);
  }

}
