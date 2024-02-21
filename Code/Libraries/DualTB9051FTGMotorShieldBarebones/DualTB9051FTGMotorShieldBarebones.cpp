#include "DualTB9051FTGMotorShieldBarebones.h"

// Constructors
DualTB9051FTGMotorShieldBarebones::DualTB9051FTGMotorShieldBarebones() {
  // Default pin assignments
  _M1EN = 2; _M1DIR = 7; _M1PWM = 9;
  _M2EN = 4; _M2DIR = 8; _M2PWM = 10;
  _M3EN = 23; _M3DIR = 27; _M3PWM = 45;
  _M4EN = 25; _M4DIR = 29; _M4PWM = 46;
  _flipM1 = _flipM2 = _flipM3 = _flipM4 = false;
}

DualTB9051FTGMotorShieldBarebones::DualTB9051FTGMotorShieldBarebones(unsigned char M3EN, unsigned char M3DIR, unsigned char M3PWM, unsigned char M4EN, unsigned char M4DIR, unsigned char M4PWM) {
  // Default pins for M1 and M2, custom for M3 and M4
  _M1EN = 2; _M1DIR = 7; _M1PWM = 9;
  _M2EN = 4; _M2DIR = 8; _M2PWM = 10;
  // User-defined pins for M3 and M4
  _M3EN = M3EN; _M3DIR = M3DIR; _M3PWM = M3PWM;
  _M4EN = M4EN; _M4DIR = M4DIR; _M4PWM = M4PWM;
  _flipM1 = _flipM2 = _flipM3 = _flipM4 = false;
}

DualTB9051FTGMotorShieldBarebones::DualTB9051FTGMotorShieldBarebones(unsigned char M1EN, unsigned char M1DIR, unsigned char M1PWM, unsigned char M2EN, unsigned char M2DIR, unsigned char M2PWM, unsigned char M3EN, unsigned char M3DIR, unsigned char M3PWM, unsigned char M4EN, unsigned char M4DIR, unsigned char M4PWM) {
  // User-defined pins for all motors
  _M1EN = M1EN; _M1DIR = M1DIR; _M1PWM = M1PWM;
  _M2EN = M2EN; _M2DIR = M2DIR; _M2PWM = M2PWM;
  _M3EN = M3EN; _M3DIR = M3DIR; _M3PWM = M3PWM;
  _M4EN = M4EN; _M4DIR = M4DIR; _M4PWM = M4PWM;
  _flipM1 = _flipM2 = _flipM3 = _flipM4 = false;
}

void DualTB9051FTGMotorShieldBarebones::init() {
  // Initialize motor control pins as outputs
  pinMode(_M1EN, OUTPUT);
  pinMode(_M1DIR, OUTPUT);
  pinMode(_M1PWM, OUTPUT);
  pinMode(_M2EN, OUTPUT);
  pinMode(_M2DIR, OUTPUT);
  pinMode(_M2PWM, OUTPUT);
  pinMode(_M3EN, OUTPUT);
  pinMode(_M3DIR, OUTPUT);
  pinMode(_M3PWM, OUTPUT);
  pinMode(_M4EN, OUTPUT);
  pinMode(_M4DIR, OUTPUT);
  pinMode(_M4PWM, OUTPUT);
}

void DualTB9051FTGMotorShieldBarebones::setM1Speed(int speed) {
  bool forward = speed > 0;
  speed = abs(speed);
  if (speed > 400) speed = 400;
  analogWrite(_M1PWM, map(speed, 0, 400, 0, 255));
  digitalWrite(_M1DIR, forward ^ _flipM1 ? HIGH : LOW);
}

void DualTB9051FTGMotorShieldBarebones::setM2Speed(int speed) {
  bool forward = speed > 0;
  speed = abs(speed);
  if (speed > 400) speed = 400;
  analogWrite(_M2PWM, map(speed, 0, 400, 0, 255));
  digitalWrite(_M2DIR, forward ^ _flipM2 ? HIGH : LOW);
}

void DualTB9051FTGMotorShieldBarebones::setM3Speed(int speed) {
  bool forward = speed > 0;
  speed = abs(speed);
  if (speed > 400) speed = 400;
  analogWrite(_M3PWM, map(speed, 0, 400, 0, 255));
  digitalWrite(_M3DIR, forward ^ _flipM3 ? HIGH : LOW);
}

void DualTB9051FTGMotorShieldBarebones::setM4Speed(int speed) {
  bool forward = speed > 0;
  speed = abs(speed);
  if (speed > 400) speed = 400;
  analogWrite(_M4PWM, map(speed, 0, 400, 0, 255));
  digitalWrite(_M4DIR, forward ^ _flipM4 ? HIGH : LOW);
}

void DualTB9051FTGMotorShieldBarebones::setSpeeds(int m1Speed, int m2Speed, int m3Speed, int m4Speed) {
  setM1Speed(m1Speed);
  setM2Speed(m2Speed);
  setM3Speed(m3Speed);
  setM4Speed(m4Speed);
}

void DualTB9051FTGMotorShieldBarebones::flipM1(boolean flip) {
  _flipM1 = flip;
}

void DualTB9051FTGMotorShieldBarebones::flipM2(boolean flip) {
  _flipM2 = flip;
}

void DualTB9051FTGMotorShieldBarebones::flipM3(boolean flip) {
  _flipM3 = flip;
}

void DualTB9051FTGMotorShieldBarebones::flipM4(boolean flip) {
  _flipM4 = flip;
}

void DualTB9051FTGMotorShieldBarebones::enableM1Driver() {
  digitalWrite(_M1EN, HIGH);
}

void DualTB9051FTGMotorShieldBarebones::enableM2Driver() {
  digitalWrite(_M2EN, HIGH);
}

void DualTB9051FTGMotorShieldBarebones::enableM3Driver() {
  digitalWrite(_M3EN, HIGH);
}

void DualTB9051FTGMotorShieldBarebones::enableM4Driver() {
  digitalWrite(_M4EN, HIGH);
}

void DualTB9051FTGMotorShieldBarebones::disableM1Driver() {
  digitalWrite(_M1EN, LOW);
}

void DualTB9051FTGMotorShieldBarebones::disableM2Driver() {
  digitalWrite(_M2EN, LOW);
}

void DualTB9051FTGMotorShieldBarebones::disableM3Driver() {
  digitalWrite(_M3EN, LOW);
}

void DualTB9051FTGMotorShieldBarebones::disableM4Driver() {
  digitalWrite(_M4EN, LOW);
}

void DualTB9051FTGMotorShieldBarebones::enableDrivers() {
  enableM1Driver();
  enableM2Driver();
  enableM3Driver();
  enableM4Driver();
}

void DualTB9051FTGMotorShieldBarebones::disableDrivers() {
  disableM1Driver();
  disableM2Driver();
  disableM3Driver();
  disableM4Driver();
}
