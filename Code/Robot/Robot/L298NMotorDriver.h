// L298NMotorDriver.h

#ifndef L298NMotorDriver_h
#define L298NMotorDriver_h

#include "Arduino.h"

class L298NMotorDriver {
public:
    L298NMotorDriver(int in1, int in2, int in3, int in4, int enA, int enB)
    : _in1(in1), _in2(in2), _in3(in3), _in4(in4), _enA(enA), _enB(enB) {}

    void init() {
        pinMode(_in1, OUTPUT);
        pinMode(_in2, OUTPUT);
        pinMode(_in3, OUTPUT);
        pinMode(_in4, OUTPUT);
        pinMode(_enA, OUTPUT);
        pinMode(_enB, OUTPUT);
    }

    void setMotorA(int speed, bool direction) {
        analogWrite(_enA, speed);
        digitalWrite(_in1, direction);
        digitalWrite(_in2, !direction);
    }

    void setMotorB(int speed, bool direction) {
        analogWrite(_enB, speed);
        digitalWrite(_in3, direction);
        digitalWrite(_in4, !direction);
    }

    void stopMotorA() {
        analogWrite(_enA, 0);
    }

    void stopMotorB() {
        analogWrite(_enB, 0);
    }

private:
    int _in1, _in2, _in3, _in4, _enA, _enB;
};

#endif
