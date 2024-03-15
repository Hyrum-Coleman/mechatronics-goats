#ifndef Pose_h
#define Pose_h

#include <Arduino.h>

class Pose {
private:
  float x, y, theta;
  unsigned long lastUpdateTime;

public:
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

  // Getters
  float get_x() const { return x; }
  float get_y() const { return y; }
  float get_theta() const { return theta; }

  // 'setter'
  void reset_pose(float newX = 0.0, float newY = 0.0, float newTheta = 0.0) {
    x = newX;
    y = newY;
    theta = newTheta;
  }
};

#endif
