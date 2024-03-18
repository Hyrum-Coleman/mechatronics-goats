#ifndef Velocities_h
#define Velocities_h

#include <Arduino.h>

class Pose;

class Velocities {
public:
  float xDot;
  float yDot;
  float thetaDot;

  Velocities(float x = 0, float y = 0, float theta = 0)
    : xDot(x), yDot(y), thetaDot(theta) {}

  // Update velocities based on a goal pose and drive time
  void calculatePathVelocities(const Pose& gRobotPose, const Pose& goalPose, float driveTime) {
    xDot = (goalPose.x - gRobotPose.x) / driveTime;
    yDot = (goalPose.y - gRobotPose.y) / driveTime;

    // Calculate angle difference considering wraparound
    float angleDifference = fmod(goalPose.theta - gRobotPose.theta + 3 * M_PI, 2 * M_PI) - M_PI;
    thetaDot = angleDifference / driveTime;
  }
  
  /* THIS FUNCTION ALMOST WORKS BUT KINDA SUCKS...
  // scales down velocity if we are close to goal
  void scale(const Pose& gRobotPose, const Pose& initialRobotPose, const Pose& goalPose) {
    float initialDistanceToGoal = initialRobotPose.getDistanceToOtherPose(goalPose);
    float currentDistToGoal = gRobotPose.getDistanceToOtherPose(goalPose);
    float xyProgress;
    if (abs(initialDistanceToGoal) < 0.2) {  // to prevent div/0 explosions
      xyProgress = 1.0;
    } else {
      xyProgress = (initialDistanceToGoal - currentDistToGoal) / initialDistanceToGoal;
    }
    Serial2.print("Positional progress: ");
    Serial2.println(xyProgress);
    float progressThresh = 0.7;  // 70 Percent
    if (xyProgress > progressThresh) {
      float minVelocity = 2.0;  // Dont go slower than 2 inches per second because we might not reach the goal.
      // Linearly interpolate velocity based on progress.
      // When progress is 0.9, use the current velocity.
      // When progress reaches 1 (or close to it), reduce velocity to minVelocity.
      float scaleFactor = 1.0 - (xyProgress - progressThresh);  // This scales from 1 to 0 as progress goes from 0.9 to 1.
      xDot = minVelocity + (xDot - minVelocity) * scaleFactor;
      yDot = minVelocity + (yDot - minVelocity) * scaleFactor;

      Serial2.print("Positional scale factor: ");
      Serial2.println(scaleFactor);
    }

    float initialAngularDistanceToGoal = initialRobotPose.getAngularDistanceToOtherPose(goalPose);
    float currentAngularDistanceToGoal = gRobotPose.getAngularDistanceToOtherPose(goalPose);
    float angularProgress;
    if (abs(initialAngularDistanceToGoal) < 0.05) {  // to prevent div/0 explosions
      angularProgress = 1.0;
    } else {
      angularProgress = (initialAngularDistanceToGoal - currentAngularDistanceToGoal) / initialAngularDistanceToGoal;
    }
        Serial2.print("Angular progress: ");
    Serial2.println(angularProgress);
    if (angularProgress > progressThresh) {  // 70 percent of the way to the angular goal
      float minAngularVelocity = 0.4;        // Don't go slower than this 0.4 rad/s

      // Similar to the linear velocity scaling, but for angular velocity.
      float angularScaleFactor = 1.0 - (angularProgress - progressThresh);
          Serial2.print("Angular Scale Factor: ");
    Serial2.println(angularScaleFactor);
      // Ensure angularScaleFactor does not go negative.
      thetaDot = minAngularVelocity + (thetaDot - minAngularVelocity) * angularScaleFactor;
    }

  }*/

  // Rotates a velocity to point from pose 1 to pose 2
  void rotate(const Pose& pose1, const Pose& pose2) {
    // Calculate the direction from pose1 to pose2
    float deltaX = pose2.x - pose1.x;
    float deltaY = pose2.y - pose1.y;

    // Calculate the new direction angle
    float angle = atan2(deltaY, deltaX);

    // Calculate the original magnitude of the velocity
    float magnitude = sqrt(xDot * xDot + yDot * yDot);

    // Update velocity to have the same magnitude but new direction
    xDot = magnitude * cos(angle);
    yDot = magnitude * sin(angle);

    // Calculate angle difference and account for wraparound
    float angleDifference = fmod(pose2.theta - pose1.theta + 3 * M_PI, 2 * M_PI) - M_PI;

    // Adjust thetaDot sign based on the sign of the angle difference
    // If angleDifference is positive, thetaDot should be positive, and vice versa
    if (angleDifference != 0) {
      thetaDot = fabs(thetaDot) * (angleDifference / fabs(angleDifference));
    }
  }
};

#endif
