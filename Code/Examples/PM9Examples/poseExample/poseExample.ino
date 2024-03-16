#include "EncoderManager.h"
#include "Pose.h"
#include "MotorRamp.h"
#include "WheelBase.h"
#include <DualTB9051FTGMotorShieldMod3230.h>
#include <Arduino.h>

struct Velocities {
  float xDot;
  float yDot;
  float thetaDot;
};

const int cEncoderCountsPerRev = 64;
const int cWheelMotorGearRatio = 50;

EncoderManager gWheel1Manager(18, 22, cEncoderCountsPerRev, cWheelMotorGearRatio, true);  // flip m1 so fwd = plus counts
EncoderManager gWheel2Manager(3, 24, cEncoderCountsPerRev, cWheelMotorGearRatio);
EncoderManager gWheel3Manager(2, 26, cEncoderCountsPerRev, cWheelMotorGearRatio, true);  // flip m3 so fwd = plus counts
EncoderManager gWheel4Manager(19, 28, cEncoderCountsPerRev, cWheelMotorGearRatio);

Velocities fwdVelocities;
Pose gRobotPose;
float odomWheelSpeeds[4];

DualTB9051FTGMotorShieldMod3230 wheels;

const float cWheelRadius = 1.2795;
const float cWheelBaseLx = 5.0625;
const float cWheelBaseLy = 4.386;

Wheelbase* gWheelbase = new Wheelbase(cWheelBaseLx, cWheelBaseLy, cWheelRadius);

void setup() {
  Serial2.begin(9600);
  wheels.init();
  wheels.enableDrivers();
  gWheel1Manager.begin();
  gWheel2Manager.begin();
  gWheel3Manager.begin();
  gWheel4Manager.begin();
  wheels.setSpeeds(100, -100, 100, -100);
}


void loop() {
  odomWheelSpeeds[0] = gWheel1Manager.getWheelSpeedRadPerSec();
  odomWheelSpeeds[1] = gWheel2Manager.getWheelSpeedRadPerSec();
  odomWheelSpeeds[2] = gWheel3Manager.getWheelSpeedRadPerSec();
  odomWheelSpeeds[3] = gWheel4Manager.getWheelSpeedRadPerSec();

  //Serial2.print("Odom: ");
  //Serial2.print(odomWheelSpeeds[0]);
  //Serial2.print(" | ");
  //Serial2.print(odomWheelSpeeds[1]);
  //Serial2.print(" | ");
  //Serial2.print(odomWheelSpeeds[2]);
  //Serial2.print(" | ");
  //Serial2.println(odomWheelSpeeds[3]);

  gWheelbase->computeVelocities(odomWheelSpeeds, fwdVelocities.xDot, fwdVelocities.yDot, fwdVelocities.thetaDot);

  Serial2.print("xDot: ");
  Serial2.print(fwdVelocities.xDot);
  Serial2.print(" | yDot: ");
  Serial2.print(fwdVelocities.yDot);
  Serial2.print(" | thDot: ");
  Serial2.println(fwdVelocities.thetaDot);

  gRobotPose.update_pos(fwdVelocities.xDot, fwdVelocities.yDot, fwdVelocities.thetaDot);

  Serial2.print("xPos: ");
  Serial2.println(gRobotPose.x);
  Serial2.print("yPos: ");
  Serial2.println(gRobotPose.y);
  Serial2.print("thPos: ");
  Serial2.println(gRobotPose.theta);
}
