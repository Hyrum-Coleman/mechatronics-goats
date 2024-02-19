#include "L298NMotorDriverMega.h"


// Constructors ////////////////////////////////////////////////////////////////

//Constructor with User-Defined Pins
L298NMotorDriverMega::L298NMotorDriverMega(unsigned char M1EN,
                                                   unsigned char M1C,
                                                   unsigned char M1D,
                                                   unsigned char M2EN,
                                                   unsigned char M2C,
                                                   unsigned char M2D)
{
  _M1PWM = M1EN;
  _M1C = M1C;
  _M1D = M1D;

  _M2PWM = M2EN;
  _M2C = M2C;
  _M2D = M2D;
 
  _flipM1 = false;
  _flipM2 = false;

}

// Public Methods //////////////////////////////////////////////////////////////

void L298NMotorDriverMega::init()
{
// Define pinMode for the pins and set the frequency for timer2.

  pinMode(_M1PWM, OUTPUT);
  pinMode(_M1C, OUTPUT);
  pinMode(_M1D, OUTPUT);
  pinMode(_M2PWM, OUTPUT);
  pinMode(_M2C, OUTPUT);
  pinMode(_M2D, OUTPUT);

  Timer = 0;

#ifdef TIMERS_AVAILABLE
  if (_M1PWM == _M1PWM_TIMER1_PIN && _M2PWM == _M2PWM_TIMER1_PIN)
  {
    // Timer 1 configuration
    // prescaler: clockI/O / 1
    // outputs enabled
    // phase-correct PWM
    // top of 400
    //
    // PWM frequency calculation
    // 16MHz / 1 (prescaler) / 2 (phase-correct) / 400 (top) = 20kHz
    TCCR1A = 0b10100000;
    TCCR1B = 0b00010001;
    ICR1 = 400;
    Timer = 1;
  }
  if (_M1PWM == _M1PWM_TIMER3_PIN && _M2PWM == _M2PWM_TIMER3_PIN)
  {
    // Timer 3 configuration
    // prescaler: clockI/O / 1
    // outputs enabled
    // phase-correct PWM
    // top of 400
    //
    // PWM frequency calculation
    // 16MHz / 1 (prescaler) / 2 (phase-correct) / 400 (top) = 20kHz
    TCCR3A = 0b10100000;
    TCCR3B = 0b00010001;
    ICR3 = 400;
    Timer = 3;
  }
  if (_M1PWM == _M1PWM_TIMER4_PIN && _M2PWM == _M2PWM_TIMER4_PIN)
  {
    // Timer 4 configuration
    // prescaler: clockI/O / 1
    // outputs enabled
    // phase-correct PWM
    // top of 400
    //
    // PWM frequency calculation
    // 16MHz / 1 (prescaler) / 2 (phase-correct) / 400 (top) = 20kHz
    TCCR4A = 0b10100000;
    TCCR4B = 0b00010001;
    ICR4 = 400;
    Timer = 4;
  }
  if (_M1PWM == _M1PWM_TIMER5_PIN && _M2PWM == _M2PWM_TIMER5_PIN)
  {
    // Timer 5 configuration
    // prescaler: clockI/O / 1
    // outputs enabled
    // phase-correct PWM
    // top of 400
    //
    // PWM frequency calculation
    // 16MHz / 1 (prescaler) / 2 (phase-correct) / 400 (top) = 20kHz
    TCCR5A = 0b10100000;
    TCCR5B = 0b00010001;
    ICR5 = 400;
    Timer = 5;
  }
#endif
}

// Set speed for motor 1, speed is a number between -400 and 400
void L298NMotorDriverMega::setM1Speed(int speed)
{
  unsigned char forward = 1;

  if (speed < 0)
  {
    speed = -speed;  // Make speed a positive quantity
    forward = 0;  // Preserve the direction
  }
  if (speed > 400)  // Max PWM dutycycle
    speed = 400;
  
  if (forward ^ _flipM1) // if speed was negative or _flipM1 setting is active, but not both
  {
    digitalWrite(_M1C, HIGH);
    digitalWrite(_M1D, LOW);
  }
  else
  {
    digitalWrite(_M1C, LOW);
    digitalWrite(_M1D, HIGH);
  }

  #ifdef TIMERS_AVAILABLE
    switch (Timer) {
     case 1:
        OCR1A = speed;
	break;
     case 3:
	OCR3A = speed;
        break;
     case 4:
        OCR4A = speed;
        break;
     case 5:
        OCR5B = speed;
        break;
     default:
        analogWrite(_M1PWM,speed * 51 / 80); // map 400 to 255
    }
  #else
    analogWrite(_M1PWM,speed * 51 / 80); // map 400 to 255
  #endif
 
}

// Set speed for motor 2, speed is a number between -400 and 400
void L298NMotorDriverMega::setM2Speed(int speed)
{
  unsigned char forward = 1;

  if (speed < 0)
  {
    speed = -speed;  // Make speed a positive quantity
    forward = 0;  // Preserve the direction
  }
  if (speed > 400)  // Max PWM dutycycle
    speed = 400;
  
  if (forward ^ _flipM2) // if speed was negative or _flipM1 setting is active, but not both
  {
    digitalWrite(_M2C, HIGH);
    digitalWrite(_M2D, LOW);
  }
  else
  {
    digitalWrite(_M2C, LOW);
    digitalWrite(_M2D, HIGH);
  }

  #ifdef TIMERS_AVAILABLE
    switch (Timer) {
     case 1:
        OCR1B = speed;
	break;
     case 3:
	OCR3B = speed;
        break;
     case 4:
        OCR4B = speed;
        break;
     case 5:
        OCR5A = speed;
        break;
     default:
        analogWrite(_M2PWM,speed * 51 / 80); // map 400 to 255
    }
  #else
    analogWrite(_M2PWM,speed * 51 / 80); // map 400 to 255
  #endif
 
}


// Set speed for both motors
void L298NMotorDriverMega::setSpeeds(int m1Speed, int m2Speed)
{
  setM1Speed(m1Speed);
  setM2Speed(m2Speed);
}

// Flip direction for motor 1
void L298NMotorDriverMega::flipM1(boolean flip)
{
  _flipM1 = flip;
}

// Flip direction for motor 2
void L298NMotorDriverMega::flipM2(boolean flip)
{
  _flipM2 = flip;
}

// Set brake for motor 1, brake is a number between 0 and 400, 0 corresponds to full coast, 400 corresponds to full brake
void L298NMotorDriverMega::setM1Brake(int brake)
{
    digitalWrite(_M1C, LOW);
    digitalWrite(_M1D, LOW);

    if (brake<0)
	brake = -brake;
    if (brake>400)
        brake = 400;

    #ifdef TIMERS_AVAILABLE
    switch (Timer) {
     case 1:
        OCR1A = brake;
	break;
     case 3:
	OCR3A = brake;
        break;
     case 4:
        OCR4A = brake;
        break;
     case 5:
        OCR5B = brake;
        break;
     default:
        analogWrite(_M1PWM,brake * 51 / 80); // map 400 to 255
    }
  #else
    analogWrite(_M1PWM,brake * 51 / 80); // map 400 to 255
  #endif
}

// Set brake for motor 2, brake is a number between 0 and 400, 0 corresponds to full coast, 400 corresponds to full brake
void L298NMotorDriverMega::setM2Brake(int brake)
{
    digitalWrite(_M2C, LOW);
    digitalWrite(_M2D, LOW);

    if (brake<0)
	brake = -brake;
    if (brake>400)
        brake = 400;

    #ifdef TIMERS_AVAILABLE
    switch (Timer) {
     case 1:
        OCR1B = brake;
	break;
     case 3:
	OCR3B = brake;
        break;
     case 4:
        OCR4B = brake;
        break;
     case 5:
        OCR5A = brake;
        break;
     default:
        analogWrite(_M2PWM,brake * 51 / 80); // map 400 to 255
    }
  #else
    analogWrite(_M2PWM,brake * 51 / 80); // map 400 to 255
  #endif
}

void L298NMotorDriverMega::setBrakes(int m1Brake, int m2Brake)
{
	setM1Brake(m1Brake);
	setM2Brake(m2Brake);
}