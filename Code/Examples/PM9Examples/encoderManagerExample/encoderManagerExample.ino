#include "EncoderManager.h"
#include <Arduino.h>

const int cEncoderCountsPerRev = 64;
const int cWheelMotorGearRatio = 50;

EncoderManager gWheel1Manager(18, 22, cEncoderCountsPerRev, cWheelMotorGearRatio, true);  // flip m1 so fwd = plus counts
EncoderManager gWheel2Manager(3, 24, cEncoderCountsPerRev, cWheelMotorGearRatio);
EncoderManager gWheel3Manager(2, 26, cEncoderCountsPerRev, cWheelMotorGearRatio, true);  // flip m3 so fwd = plus counts
EncoderManager gWheel4Manager(19, 28, cEncoderCountsPerRev, cWheelMotorGearRatio);

void setup() {
  Serial.begin(9600);
  gWheel1Manager.begin();
  gWheel2Manager.begin();
  gWheel3Manager.begin();
  gWheel4Manager.begin();
}


void loop() {
  // Read and print the wheel speed every second
  delay(200); // Wait for 1 second
  
  float speedCPS1 = gWheel1Manager.getWheelSpeedCPS();
  float speedRadPerSec1 = gWheel1Manager.getWheelSpeedRadPerSec();

  float speedCPS2 = gWheel2Manager.getWheelSpeedCPS();
  float speedRadPerSec2 = gWheel2Manager.getWheelSpeedRadPerSec();

  float speedCPS3 = gWheel3Manager.getWheelSpeedCPS();
  float speedRadPerSec3 = gWheel3Manager.getWheelSpeedRadPerSec();

  float speedCPS4 = gWheel4Manager.getWheelSpeedCPS();
  float speedRadPerSec4 = gWheel4Manager.getWheelSpeedRadPerSec();

  // Print all wheel speeds on one line, separated by '|'
  Serial.print("W1: CPS=");
  Serial.print(speedCPS1, 2);
  Serial.print(", Rad/s=");
  Serial.print(speedRadPerSec1, 2);
  Serial.print(" | W2: CPS=");
  Serial.print(speedCPS2, 2);
  Serial.print(", Rad/s=");
  Serial.print(speedRadPerSec2, 2);
  Serial.print(" | W3: CPS=");
  Serial.print(speedCPS3, 2);
  Serial.print(", Rad/s=");
  Serial.print(speedRadPerSec3, 2);
  Serial.print(" | W4: CPS=");
  Serial.print(speedCPS4, 2);
  Serial.print(", Rad/s=");
  Serial.println(speedRadPerSec4, 2);
}
