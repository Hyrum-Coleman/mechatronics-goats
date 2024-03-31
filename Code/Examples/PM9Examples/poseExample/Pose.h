#ifndef Pose_h
#define Pose_h

#include <Arduino.h>

class Pose {
private:
  unsigned long lastUpdateTime;

public:
  // Properties
  float x, y, theta;

  // constructor
  Pose(float initX = 0.0, float initY = 0.0, float initTheta = 0.0)
      : x(initX), y(initY), theta(initTheta), lastUpdateTime(millis()) {}

  void update_pos(float x_dot, float y_dot, float theta_dot) {
    unsigned long currentTime = millis();
    float dt = (currentTime - lastUpdateTime) /
               1000.0; // Convert milliseconds to seconds
    x += x_dot * dt;
    y += y_dot * dt;
    theta += theta_dot * dt;

    lastUpdateTime = currentTime; // Update the last update time
  }

  // 'setter'
  void reset_pose(float newX = 0.0, float newY = 0.0, float newTheta = 0.0) {
    x = newX;
    y = newY;
    theta = newTheta;
  }
};

#endif
