#ifndef Pose_h
#define Pose_h

#include <Arduino.h>

class Pose {
private:
  double lastUpdateTime;

public:
  // Properties
  float x, y, theta;

  // constructor
  Pose(float initX = 0.0, float initY = 0.0, float initTheta = 0.0)
    : x(initX), y(initY), theta(initTheta), lastUpdateTime((double)micros() / 1000000.) {}

  void update_pos(float x_dot, float y_dot, float theta_dot) {
    double currentTime = (double)micros() / 1000000.; // in s
    float dt = (currentTime - lastUpdateTime); 
    x += x_dot * dt;
    y += y_dot * dt;
    theta += theta_dot * dt;

    lastUpdateTime = currentTime;  // Update the last update time
  }

  void rotate(float angleRadians) {
    float cosTheta = cos(angleRadians);
    float sinTheta = sin(angleRadians);

    float newX = x * cosTheta - y * sinTheta;
    float newY = x * sinTheta + y * cosTheta;

    x = newX;
    y = newY;
  }

  // Compares to another pose
  bool aligned(const Pose& otherPose, float threshold_inches, float threshold_rad) const {
    float diffX = abs(x - otherPose.x);
    float diffY = abs(y - otherPose.y);
    float diffTheta = abs(theta - otherPose.theta);

    return (diffX <= threshold_inches) && (diffY <= threshold_inches) && (diffTheta <= threshold_rad);
  }

  // Compares to another pose, but only in a certain direction (IE, the direction youre driving).
  // This is a workaround to gradual error in the non-driving direction that builds up due to noise, making aligned() never return true.
  bool alignedInErrorDir(const Pose& otherPose, float threshold, float dirX, float dirY, float dirTheta) const {
    float diffX = x - otherPose.x;
    float diffY = y - otherPose.y;
    float diffTheta = theta - otherPose.theta;
    
    float mag = sqrt(dirX * dirX + dirY * dirY + dirTheta * dirTheta);

    if (mag == 0) return false;

    dirX /= mag;
    dirY /= mag;
    dirTheta /= mag;

    float adjustedThreshold = threshold * (1-0.7*dirTheta); // adjusts for the units of theta being larger proportionally than inches

    float projection = diffX * dirX + diffY * dirY + diffTheta * dirTheta;

    return abs(projection) <= adjustedThreshold;
  }

  float getDistanceToOtherPose(const Pose& otherPose) {
    float dx = otherPose.x - x;
    float dy = otherPose.y - y;

    return sqrt(dx*dx + dy*dy);
  }

  float getAngularDistanceToOtherPose(const Pose& otherPose) {
    float dth = otherPose.theta - theta;
    return dth;
  }


  // 'setter'
  void reset_pose(float newX = 0.0, float newY = 0.0, float newTheta = 0.0) {
    x = newX;
    y = newY;
    theta = newTheta;
  }
};

#endif
