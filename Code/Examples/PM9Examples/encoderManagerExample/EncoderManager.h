#ifndef EncoderManager_h
#define EncoderManager_h

#include <Arduino.h>
#include <Encoder.h>

class EncoderManager {
private:
  Encoder enc;
  long prevCountCPS, prevCountRadPerSec;
  unsigned long prevTimeCPS, prevTimeRadPerSec;
  float countsPerRev;
  float gearRatio;
  bool flipped;

public:
  EncoderManager(uint8_t pin1, uint8_t pin2, float cpr, float gr, bool flip = false)
    : enc(pin1, pin2), prevCountCPS(0), prevCountRadPerSec(0),
      prevTimeCPS(0), prevTimeRadPerSec(0), countsPerRev(cpr), gearRatio(gr), flipped(flip) {}

  void begin() {
    long currentCount = enc.read();
    unsigned long currentTime = millis();
    prevCountCPS = currentCount;
    prevCountRadPerSec = currentCount;
    prevTimeCPS = currentTime;
    prevTimeRadPerSec = currentTime;
  }

  void reset() {
    enc.write(0);
    begin();
  }

  float getWheelSpeedCPS() {
    long currentCount = enc.read();
    if (flipped) currentCount = -currentCount;
    unsigned long currentTime = millis();
    float speed = 0.0;

    if (currentTime != prevTimeCPS) {
      speed = (currentCount - prevCountCPS) / ((currentTime - prevTimeCPS) / 1000.0);
    }

    prevCountCPS = currentCount;
    prevTimeCPS = currentTime;

    return speed;
  }

  float getWheelSpeedRadPerSec() {
    long currentCount = enc.read();
    if (flipped) currentCount = -currentCount;
    unsigned long currentTime = millis();
    float deltaCount = currentCount - prevCountRadPerSec;
    float deltaTime = (currentTime - prevTimeRadPerSec) / 1000.0;

    float theta = (deltaCount * 2 * PI) / (countsPerRev * gearRatio);
    float speedRadPerSec = deltaTime > 0 ? theta / deltaTime : 0;

    prevCountRadPerSec = currentCount;
    prevTimeRadPerSec = currentTime;

    return speedRadPerSec;
  }
};

#endif
