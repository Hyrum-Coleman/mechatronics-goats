#include "DualTB9051FTGMotorShieldMod3230.h"

// If using the default pins, use the following line:
DualTB9051FTGMotorShieldMod3230 md;

// If using custom pins for motors 3 and 4 use the following line:
// You will need to pick what M3EN,M3DIR,... are.
// DualTB9051FTGMotorShieldMod3230 md(M3EN,M3DIR,M3PWM,M3DIAG,M3OCM,M4EN,M4DIR,M4PWM,M4DIAG,M4OCM);


float f = 0.25;             //frequency in Hz
unsigned long t = 0;        //current time

void setup() {
  // put your setup code here, to run once:
  
  md.init();
  md.enableDrivers();
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
    md.setM1Speed(0);
  }
  // Turn on Motor 2 for 4 seconds /////////////////////////////////////////
  if (4.0 < t && t < 8.0) {
    md.setM2Speed(M);
  }
  else {
    md.setM2Speed(0);
  }

  // Turn on Motor 3 for 4 seconds /////////////////////////////////////////
  if (8.0 < t && t < 12.0) {
    md.setM3Speed(M);
  }
  else {
    md.setM3Speed(0);
  }

  // Turn on Motor 4 for 4 seconds /////////////////////////////////////////
  if (12.0 < t && t < 16.0) {
    md.setM4Speed(M);
  }
  else {
    md.setM4Speed(0);
  }

}
