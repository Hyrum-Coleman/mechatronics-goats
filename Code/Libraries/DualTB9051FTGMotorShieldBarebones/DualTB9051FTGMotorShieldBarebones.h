#ifndef DualTB9051FTGMotorShieldBarebones_h
#define DualTB9051FTGMotorShieldBarebones_h

#include "Arduino.h"

class DualTB9051FTGMotorShieldBarebones
{
  public:
    // Constructors
    DualTB9051FTGMotorShieldBarebones();
    DualTB9051FTGMotorShieldBarebones(unsigned char M3EN, unsigned char M3DIR, unsigned char M3PWM, unsigned char M4EN, unsigned char M4DIR, unsigned char M4PWM);
    DualTB9051FTGMotorShieldBarebones(unsigned char M1EN, unsigned char M1DIR, unsigned char M1PWM, unsigned char M2EN, unsigned char M2DIR, unsigned char M2PWM, unsigned char M3EN, unsigned char M3DIR, unsigned char M3PWM, unsigned char M4EN, unsigned char M4DIR, unsigned char M4PWM);

    // Public Methods
    void init(); // Initialization
    void setM1Speed(int speed); // Set speed for motor 1
    void setM2Speed(int speed); // Set speed for motor 2
    void setM3Speed(int speed); // Set speed for motor 3
    void setM4Speed(int speed); // Set speed for motor 4
    void setSpeeds(int m1Speed, int m2Speed, int m3Speed, int m4Speed); // Set speed for all motors
    void flipM1(boolean flip); // Flip direction for motor 1
    void flipM2(boolean flip); // Flip direction for motor 2
    void flipM3(boolean flip); // Flip direction for motor 3
    void flipM4(boolean flip); // Flip direction for motor 4
    void enableM1Driver(); // Enable the driver for motor 1
    void enableM2Driver(); // Enable the driver for motor 2
    void enableM3Driver(); // Enable the driver for motor 3
    void enableM4Driver(); // Enable the driver for motor 4
    void disableM1Driver(); // Disable the driver for motor 1
    void disableM2Driver(); // Disable the driver for motor 2
    void disableM3Driver(); // Disable the driver for motor 3
    void disableM4Driver(); // Disable the driver for motor 4
    void enableDrivers(); // Enable the drivers for all motors
    void disableDrivers(); // Disable the drivers for all motors

  private:
    // Private Members
    unsigned char _M1EN, _M1DIR, _M1PWM;
    unsigned char _M2EN, _M2DIR, _M2PWM;
    unsigned char _M3EN, _M3DIR, _M3PWM;
    unsigned char _M4EN, _M4DIR, _M4PWM;
    boolean _flipM1, _flipM2, _flipM3, _flipM4; // Direction flip flags
};

#endif
