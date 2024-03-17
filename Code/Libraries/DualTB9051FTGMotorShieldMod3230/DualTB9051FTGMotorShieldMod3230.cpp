#include "DualTB9051FTGMotorShieldMod3230.h"



// Constructors ////////////////////////////////////////////////////////////////

//Default Constructor
DualTB9051FTGMotorShieldMod3230::DualTB9051FTGMotorShieldMod3230()
{
  //Pin map
  _M1EN = 12;
  _M1DIR = 7;
  _M1PWM = 9;
  _M1DIAG = 99;
  _M1OCM = A9;

  _M2EN = 4;
  _M2DIR = 8;
  _M2PWM = 10;
  _M2DIAG = 99;
  _M2OCM = A9;

  _M3EN = 23;
  _M3DIR = 27;
  _M3PWM = 45;
  _M3DIAG = 99;
  _M3OCM = A9;

  _M4EN = 25;
  _M4DIR = 29;
  _M4PWM = 46;
  _M4DIAG = 99;
  _M4OCM = A9;

  _flipM1 = false;
  _flipM2 = false;
  _flipM3 = false;
  _flipM4 = false;
}


//Constructor with Default Pins for Motors1&2, User-Defined Pins for Motors 3&4
DualTB9051FTGMotorShieldMod3230::DualTB9051FTGMotorShieldMod3230(unsigned char M3EN,
                                                   unsigned char M3DIR,
                                                   unsigned char M3PWM,
                                                   unsigned char M3DIAG,
                                                   unsigned char M3OCM,
                                                   unsigned char M4EN,
                                                   unsigned char M4DIR,
                                                   unsigned char M4PWM,
                                                   unsigned char M4DIAG,
                                                   unsigned char M4OCM)
{
  _M1EN = 2;
  _M1DIR = 7;
  _M1PWM = 9;
  _M1DIAG = 6;
  _M1OCM = A0;

  _M2EN = 4;
  _M2DIR = 8;
  _M2PWM = 10;
  _M2DIAG = 12;
  _M2OCM = A1;

  _M3EN = M3EN;
  _M3DIR = M3DIR;
  _M3PWM = M3PWM;
  _M3DIAG = M3DIAG;
  _M3OCM = M3OCM;

  _M4EN = M4EN;
  _M4DIR = M4DIR;
  _M4PWM = M4PWM;
  _M4DIAG = M4DIAG;
  _M4OCM = M4OCM;

  _flipM1 = false;
  _flipM2 = false;
  _flipM3 = false;
  _flipM4 = false;
}

//Constructor with User-Defined Pins for All Motors
DualTB9051FTGMotorShieldMod3230::DualTB9051FTGMotorShieldMod3230(unsigned char M1EN,
                                                   unsigned char M1DIR,
                                                   unsigned char M1PWM,
                                                   unsigned char M1DIAG,
                                                   unsigned char M1OCM,
                                                   unsigned char M2EN,
                                                   unsigned char M2DIR,
                                                   unsigned char M2PWM,
                                                   unsigned char M2DIAG,
                                                   unsigned char M2OCM,
                                                   unsigned char M3EN,
                                                   unsigned char M3DIR,
                                                   unsigned char M3PWM,
                                                   unsigned char M3DIAG,
                                                   unsigned char M3OCM,
                                                   unsigned char M4EN,
                                                   unsigned char M4DIR,
                                                   unsigned char M4PWM,
                                                   unsigned char M4DIAG,
                                                   unsigned char M4OCM)
{
  _M1EN = M1EN;
  _M1DIR = M1DIR;
  _M1PWM = M1PWM;
  _M1DIAG = M1DIAG;
  _M1OCM = M1OCM;

  _M2EN = M2EN;
  _M2DIR = M2DIR;
  _M2PWM = M2PWM;
  _M2DIAG = M2DIAG;
  _M2OCM = M2OCM;

  _M3EN = M3EN;
  _M3DIR = M3DIR;
  _M3PWM = M3PWM;
  _M3DIAG = M3DIAG;
  _M3OCM = M3OCM;

  _M4EN = M4EN;
  _M4DIR = M4DIR;
  _M4PWM = M4PWM;
  _M4DIAG = M4DIAG;
  _M4OCM = M4OCM;

  _flipM1 = false;
  _flipM2 = false;
  _flipM3 = false;
  _flipM4 = false;
}

// Public Methods //////////////////////////////////////////////////////////////

void DualTB9051FTGMotorShieldMod3230::init()
{
// Define pinMode for the pins and set the frequency for timer1/timer2.

  pinMode(_M1EN, OUTPUT);
  pinMode(_M2EN, OUTPUT);
  pinMode(_M1DIR, OUTPUT);
  pinMode(_M1PWM, OUTPUT);
  pinMode(_M1DIAG, INPUT_PULLUP);
  pinMode(_M1OCM, INPUT);
  pinMode(_M2DIR, OUTPUT);
  pinMode(_M2PWM, OUTPUT);
  pinMode(_M2DIAG, INPUT_PULLUP);
  pinMode(_M2OCM, INPUT);

#ifdef DUALTB9051FTGMOTORSHIELD_TIMER1_AVAILABLE
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
  }
#endif
#ifdef DUALTB9051FTGMOTORSHIELD_TIMER2_AVAILABLE
	if (_M1PWM == _M1PWM_TIMER2_PIN && _M2PWM == _M2PWM_TIMER2_PIN)
	{
		// Timer 2 configuration
		// prescaler: clockI/O / 8
		// outputs enabled
		// fast PWM
		// top of 255
		//
		// PWM frequency calculation
		// 16MHz / 8 (prescaler) / 1 (fast PWM) / 255 (top) = 7.8kHz
		TCCR2A = 0b10100011; //11 in first two bits = fast PWM (could switch to phase correct PWM by making first two bits = 01, PWM frequency would be 3.9kHz)
		TCCR2B = 0b00000010; //first 3 bits are prescaler
	}
  #endif
// Define pinMode for the pins and set the frequency for timer5.

  pinMode(_M3EN, OUTPUT);
  pinMode(_M4EN, OUTPUT);
  pinMode(_M3DIR, OUTPUT);
  pinMode(_M3PWM, OUTPUT);
  pinMode(_M3DIAG, INPUT_PULLUP);
  pinMode(_M3OCM, INPUT);
  pinMode(_M4DIR, OUTPUT);
  pinMode(_M4PWM, OUTPUT);
  pinMode(_M4DIAG, INPUT_PULLUP);
  pinMode(_M4OCM, INPUT);

#ifdef DUALTB9051FTGMOTORSHIELD_TIMER5_AVAILABLE
  if (_M3PWM == _M3PWM_TIMER5_PIN && _M4PWM == _M4PWM_TIMER5_PIN)
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
  }
#endif
}

// Set speed for motor 1, speed is a number between -400 and 400
void DualTB9051FTGMotorShieldMod3230::setM1Speed(int speed)
{
  unsigned char forward = 1;

  if (speed < 0)
  {
    speed = -speed;  // Make speed a positive quantity
    forward = 0;  // Preserve the direction
  }
  if (speed > 400)  // Max PWM dutycycle
    speed = 400;

  #ifdef DUALTB9051FTGMOTORSHIELD_TIMER1_AVAILABLE
    if (_M1PWM == _M1PWM_TIMER1_PIN && _M2PWM == _M2PWM_TIMER1_PIN)
    {
      OCR1A = speed;
    }
    else
    {
      analogWrite(_M1PWM,speed * 51 / 80); // map 400 to 255
    }
  #elif defined(DUALTB9051FTGMOTORSHIELD_TIMER2_AVAILABLE)
    if (_M1PWM == _M1PWM_TIMER2_PIN && _M2PWM == _M2PWM_TIMER2_PIN)
    {
      OCR2B = speed*51/80; // map 400 to 255
    }
    else
    {
      analogWrite(_M1PWM,speed * 51 / 80); // map 400 to 255
    }
  #else
    analogWrite(_M1PWM,speed * 51 / 80); // map 400 to 255
  #endif
 
  if (forward ^ _flipM1) // DIR high if speed was negative or _flipM1 setting is active, but not both
  {
    digitalWrite(_M1DIR, HIGH);
  }
  else
  {
    digitalWrite(_M1DIR, LOW);
  }
}

// Set speed for motor 2, speed is a number between -400 and 400
void DualTB9051FTGMotorShieldMod3230::setM2Speed(int speed)
{
  unsigned char forward = 1;

  if (speed < 0)
  {
    speed = -speed;  // Make speed a positive quantity
    forward = 0;  // Preserve the direction
  }
  if (speed > 400)  // Max PWM dutycycle
    speed = 400;

  #ifdef DUALTB9051FTGMOTORSHIELD_TIMER1_AVAILABLE
    if (_M1PWM == _M1PWM_TIMER1_PIN && _M2PWM == _M2PWM_TIMER1_PIN)
    {
      OCR1B = speed;
    }
    else
    {
      analogWrite(_M2PWM,speed * 51 / 80); // map 400 to 255
    }
  #elif defined(DUALTB9051FTGMOTORSHIELD_TIMER2_AVAILABLE)
    if (_M1PWM == _M1PWM_TIMER2_PIN && _M2PWM == _M2PWM_TIMER2_PIN)
    {
      OCR2A = speed*51/80; // map 400 to 255
    }
    else
    {
      analogWrite(_M2PWM,speed * 51 / 80); // map 400 to 255
    }
  #else
    analogWrite(_M2PWM,speed * 51 / 80); // map 400 to 255
  #endif

  if (forward ^ _flipM2) // DIR high if speed was negative or _flipM2 setting is active, but not both
  {
    digitalWrite(_M2DIR, HIGH);

  }
  else
  {
    digitalWrite(_M2DIR, LOW);

  }
}
// Set speed for motor 3, speed is a number between -400 and 400
void DualTB9051FTGMotorShieldMod3230::setM3Speed(int speed)
{
  unsigned char forward = 1;

  if (speed < 0)
  {
    speed = -speed;  // Make speed a positive quantity
    forward = 0;  // Preserve the direction
  }
  if (speed > 400)  // Max PWM dutycycle
    speed = 400;

  #ifdef DUALTB9051FTGMOTORSHIELD_TIMER5_AVAILABLE
    if (_M3PWM == _M3PWM_TIMER5_PIN && _M4PWM == _M4PWM_TIMER5_PIN)
    {
      OCR5B = speed;
    }
    else
    {
      analogWrite(_M3PWM,speed * 51 / 80); // map 400 to 255
    }
  #else
    analogWrite(_M3PWM,speed * 51 / 80); // map 400 to 255
  #endif
  
  if (forward ^ _flipM3) // DIR high if speed was negative or _flipM3 setting is active, but not both
  {
    digitalWrite(_M3DIR, HIGH);
  }
  else
  {
    digitalWrite(_M3DIR, LOW);
  }
}
// Set speed for motor 4, speed is a number between -400 and 400
void DualTB9051FTGMotorShieldMod3230::setM4Speed(int speed)
{
  unsigned char forward = 1;

  if (speed < 0)
  {
    speed = -speed;  // Make speed a positive quantity
    forward = 0;  // Preserve the direction
  }
  if (speed > 400)  // Max PWM dutycycle
    speed = 400;

  #ifdef DUALTB9051FTGMOTORSHIELD_TIMER5_AVAILABLE

    if (_M3PWM == _M3PWM_TIMER5_PIN && _M4PWM == _M4PWM_TIMER5_PIN)
    {
      OCR5A = speed;

    }
    else
    {
      analogWrite(_M4PWM,speed * 51 / 80); // map 400 to 255
    }
  #else

    analogWrite(_M4PWM,speed * 51 / 80); // map 400 to 255
  #endif
  
  if (forward ^ _flipM4) // DIR high if speed was negative or _flipM4 setting is active, but not both
  {
    digitalWrite(_M4DIR, HIGH);
  }
  else
  {
    digitalWrite(_M4DIR, LOW);
  }
}

// Set speed for all motors
void DualTB9051FTGMotorShieldMod3230::setSpeeds(int m1Speed, int m2Speed, int m3Speed, int m4Speed)
{
  setM1Speed(m1Speed);
  setM2Speed(m2Speed);
  setM3Speed(m3Speed);
  setM4Speed(m4Speed);
}

// Return error status for motor 1
unsigned char DualTB9051FTGMotorShieldMod3230::getM1Fault()
{
  return !digitalRead(_M1DIAG);
}

// Return error status for motor 2
unsigned char DualTB9051FTGMotorShieldMod3230::getM2Fault()
{
  return !digitalRead(_M2DIAG);
}

// Return error status for motor 3
unsigned char DualTB9051FTGMotorShieldMod3230::getM3Fault()
{
  return !digitalRead(_M3DIAG);
}

// Return error status for motor 4
unsigned char DualTB9051FTGMotorShieldMod3230::getM4Fault()
{
  return !digitalRead(_M4DIAG);
}

// Flip direction for motor 1
void DualTB9051FTGMotorShieldMod3230::flipM1(boolean flip)
{
  DualTB9051FTGMotorShieldMod3230::_flipM1 = flip;
}

// Flip direction for motor 2
void DualTB9051FTGMotorShieldMod3230::flipM2(boolean flip)
{
  DualTB9051FTGMotorShieldMod3230::_flipM2 = flip;
}

// Flip direction for motor 3
void DualTB9051FTGMotorShieldMod3230::flipM3(boolean flip)
{
  DualTB9051FTGMotorShieldMod3230::_flipM3 = flip;
}

// Flip direction for motor 4
void DualTB9051FTGMotorShieldMod3230::flipM4(boolean flip)
{
  DualTB9051FTGMotorShieldMod3230::_flipM4 = flip;
}

// Enable the driver for motor 1
void DualTB9051FTGMotorShieldMod3230::enableM1Driver()
{
  digitalWrite(_M1EN, HIGH);
}

// Enable the driver for motor 2
void DualTB9051FTGMotorShieldMod3230::enableM2Driver()
{
  digitalWrite(_M2EN, HIGH);
}

// Enable the driver for motor 3
void DualTB9051FTGMotorShieldMod3230::enableM3Driver()
{
  digitalWrite(_M3EN, HIGH);
}

// Enable the driver for motor 4
void DualTB9051FTGMotorShieldMod3230::enableM4Driver()
{
  digitalWrite(_M4EN, HIGH);
}

// Enable the drivers for all motors
void DualTB9051FTGMotorShieldMod3230::enableDrivers()
{
  digitalWrite(_M1EN, HIGH);
  digitalWrite(_M2EN, HIGH);
  digitalWrite(_M3EN, HIGH);
  digitalWrite(_M4EN, HIGH);
}

// Disable the driver for motor 1
void DualTB9051FTGMotorShieldMod3230::disableM1Driver()
{
  digitalWrite(_M1EN, LOW);
}

// Disable the driver for motor 2
void DualTB9051FTGMotorShieldMod3230::disableM2Driver()
{
  digitalWrite(_M2EN, LOW);
}

// Disable the driver for motor 3
void DualTB9051FTGMotorShieldMod3230::disableM3Driver()
{
  digitalWrite(_M3EN, LOW);
}

// Disable the driver for motor 4
void DualTB9051FTGMotorShieldMod3230::disableM4Driver()
{
  digitalWrite(_M4EN, LOW);
}

// Disable the drivers for all motors
void DualTB9051FTGMotorShieldMod3230::disableDrivers()
{
  digitalWrite(_M1EN, LOW);
  digitalWrite(_M2EN, LOW);
  digitalWrite(_M3EN, LOW);
  digitalWrite(_M4EN, LOW);
}

// Return motor 1 current value in milliamps.
unsigned int DualTB9051FTGMotorShieldMod3230::getM1CurrentMilliamps()
{
  // 5V / 1024 ADC counts / 500 mV per A = 10 mA per count
  return analogRead(_M1OCM) * 10;
}

// Return motor 2 current value in milliamps.
unsigned int DualTB9051FTGMotorShieldMod3230::getM2CurrentMilliamps()
{
  // 5V / 1024 ADC counts / 500 mV per A = 10 mA per count
  return analogRead(_M2OCM) * 10;
}

// Return motor 3 current value in milliamps.
unsigned int DualTB9051FTGMotorShieldMod3230::getM3CurrentMilliamps()
{
  // 5V / 1024 ADC counts / 500 mV per A = 10 mA per count
  return analogRead(_M3OCM) * 10;
}

// Return motor 4 current value in milliamps.
unsigned int DualTB9051FTGMotorShieldMod3230::getM4CurrentMilliamps()
{
  // 5V / 1024 ADC counts / 500 mV per A = 10 mA per count
  return analogRead(_M4OCM) * 10;
}