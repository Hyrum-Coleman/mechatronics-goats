#ifndef EncoderManager_h
#define EncoderManager_h

#include <Encoder.h>
#include <Arduino.h>

class EncoderManager {
private:
  Encoder enc;
  long prevCount;
  unsigned long prevTime;
  float countsPerRev;
  float gearRatio;
  bool flipped;

public:
  EncoderManager(uint8_t pin1, uint8_t pin2, float cpr, float gr, bool flip = false) : enc(pin1, pin2), prevCount(0), prevTime(0), countsPerRev(cpr), gearRatio(gr), flipped(flip) {}
  // Initializes.
  void begin() {
    prevCount = enc.read();
    prevTime = millis();
  }

  // Resets the count.
  void reset() {
    enc.write(0);
    begin();
  }

  // Returns the wheel's speed in Counts Per Second (CPS)
  float getWheelSpeedCPS() {
    long currentCount = enc.read();        // Read current encoder count
    if(flipped) currentCount = -currentCount;
    unsigned long currentTime = millis();  // Get current time
    float speed = 0.0;                     // Initialize speed

    // Check for div/0
    if (currentTime != prevTime) {
      speed = (currentCount - prevCount) / ((currentTime - prevTime) / 1000.0);  // Calculate speed in CPS
    }

    // Update previous count and time for next calculation
    prevCount = currentCount;
    prevTime = currentTime;

    return speed;  // Return calculated speed
  }

  // Returns the wheel's speed in Radians Per Second (Rad/s)
  float getWheelSpeedRadPerSec() {
    long currentCount = enc.read();                       // Read current encoder count
    if(flipped) currentCount = -currentCount;
    unsigned long currentTime = millis();                 // Get current time
    float deltaCount = currentCount - prevCount;          // Calculate change in count
    float deltaTime = (currentTime - prevTime) / 1000.0;  // Calculate elapsed time in seconds

    // Calculate angle in radians covered since last measurement
    float theta = (deltaCount * 2 * PI) / (countsPerRev * gearRatio);

    // Check for div/0
    float speedRadPerSec = deltaTime > 0 ? theta / deltaTime : 0;

    // Update previous count and time for next calculation
    prevCount = currentCount;
    prevTime = currentTime;

    return speedRadPerSec;  // Return calculated speed
  }
};

#endif
