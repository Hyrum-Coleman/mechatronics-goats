// MotorRamp.h
#ifndef MotorRamp_h
#define MotorRamp_h

#include "Arduino.h"

// Dont want to figure out how to scope in our global number of wheels. God forbid its ever different than 4.
class MotorRamp {
public:
    MotorRamp(const float wheelSpeeds[4], unsigned long rampTime) : _rampTime(rampTime) {
        _startTime = millis();
        for (int i = 0; i < 4; i++) {
            _goalSpeeds[i] = wheelSpeeds[i]; 
        }
    }

    void setGoalSpeeds(const float wheelSpeeds[4]) {
        for (int i = 0; i < 4; i++) {
            _goalSpeeds[i] = wheelSpeeds[i];
        }
        _startTime = millis(); // Reset start time whenever new goal speeds are set
    }

    void setRampTime(unsigned long rampTime) {
        _rampTime = rampTime;
    }

    float getCurrentSpeed(int wheelIndex) {
        unsigned long elapsedTime = millis() - _startTime;
        if (elapsedTime >= _rampTime) {
            return _goalSpeeds[wheelIndex];
        }
        return _goalSpeeds[wheelIndex] * (float(elapsedTime) / float(_rampTime));
    }

    void setControlSignals(float (&controlSignals)[4]) {
        unsigned long elapsedTime = millis() - _startTime;
        for (int i = 0; i < 4; i++) {
            if (elapsedTime >= _rampTime) {
                controlSignals[i] = _goalSpeeds[i];
            } else {
                controlSignals[i] = _goalSpeeds[i] * (float(elapsedTime) / float(_rampTime));
            }
        }
    }

private:
    unsigned long _startTime;
    float _goalSpeeds[4];
    unsigned long _rampTime;
};

#endif
