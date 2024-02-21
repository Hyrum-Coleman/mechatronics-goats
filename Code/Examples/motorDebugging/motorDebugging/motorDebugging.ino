#include <L298NMotorDriverMega.h>
#include <DualTB9051FTGMotorShieldMod3230.h>

// To try:
// 1. remove serial comms and see if wheels still move slowly
// 2. try changing motor writes from 255 to 400
// 3. look at the below comments of 'POSSIBLE PROBLEMS'

const int ENA = 5; 
const int IN1 = 34;
const int IN2 = 32;
const int ENB = 6;
const int IN3 = 33;
const int IN4 = 35;

L298NMotorDriverMega md(ENA,IN1,IN2,ENB,IN3,IN4); // create object for motor driver
DualTB9051FTGMotorShieldMod3230 wheelMotors;

float f = 0.25;             //frequency in Hz
unsigned long t = 0;        //current time
String message = "";

void setup() {
  // put your setup code here, to run once:
  md.init();

  wheelMotors.init();
  wheelMotors.enableDrivers();

  Serial.begin(9600);
  Serial2.begin(9600);
  Serial2.println("Connected!");
}

void loop() {
    // Serial Comms
    // POSSIBLE PROBLEM: Checking serial and assigning message EVERY TIME THROUGH THE LOOP. How often is it being sent? What if it reassigns a partial message?
    if (Serial2.available() > 2) {
      message = Serial2.readStringUntil('\n');
      Serial2.println(message);
    }

    if (message.equals("Go!")) {
      // put your main code here, to run repeatedly:
      // POSSIBLE PROBLEM -- SHOULD THIS BE ONLY UPDATED WHEN MESSAGE IS GO? OR SHOULD IT BE UPDATED OUTSIDE?
      double t = int(micros() / 1000000.0) % 32; //current time -- change the mod to % 32 if driving wheels
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

      // Turn wheels forwards for 2 seconds /////////////////////////////////////////
      if (8.0 < t && t < 12.0) {
        wheelMotors.setSpeeds(255, -255, 255, -255);
      }
      else {
        wheelMotors.setSpeeds(0, 0, 0, 0);
      }

      // Turn wheels backwards for 2 seconds /////////////////////////////////////////
      if (12.0 < t && t < 16.0) {
        wheelMotors.setSpeeds(-255, 255, -255, 255);
      }
      else {
        wheelMotors.setSpeeds(0, 0, 0, 0);
      }

      // Turn wheels left for 2 seconds /////////////////////////////////////////
      if (16.0 < t && t < 20.0) {
        wheelMotors.setSpeeds(255, 255, -255, -255);
      }
      else {
        wheelMotors.setSpeeds(0, 0, 0, 0);
      }

      // Turn wheels right for 2 seconds /////////////////////////////////////////
      if (20.0 < t && t < 24.0) {
        wheelMotors.setSpeeds(-255, -255, 255, 255);
      }
      else {
        wheelMotors.setSpeeds(0, 0, 0, 0);
      }

      // Turn wheels clockwise for 2 seconds /////////////////////////////////////////
      if (24.0 < t && t < 28.0) {
        wheelMotors.setSpeeds(255, 255, 255, 255);
      }
      else {
        wheelMotors.setSpeeds(0, 0, 0, 0);
      }

      // Turn wheels counterclockwise for 2 seconds /////////////////////////////////////////
      if (28.0 < t && t < 32.0) {
        wheelMotors.setSpeeds(-255, -255, -255, -255);
      }
      else {
        wheelMotors.setSpeeds(0, 0, 0, 0);
      }

    }



 }
