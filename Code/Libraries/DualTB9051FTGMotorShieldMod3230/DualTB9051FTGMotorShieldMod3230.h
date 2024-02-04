#pragma once

#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) || \
    defined(__AVR_ATmega328PB__) || defined (__AVR_ATmega32U4__)
  #define DUALTB9051FTGMOTORSHIELD_TIMER1_AVAILABLE
#endif

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  #define DUALTB9051FTGMOTORSHIELD_TIMER2_AVAILABLE
  #define DUALTB9051FTGMOTORSHIELD_TIMER5_AVAILABLE
#endif

#include <Arduino.h>

class DualTB9051FTGMotorShieldMod3230
{
  public:
    // CONSTRUCTORS
    // Default pin selection.
    DualTB9051FTGMotorShieldMod3230();
    // Default pin selection for motors 1&2, user-defined pin selection for motors 3&4
    DualTB9051FTGMotorShieldMod3230(unsigned char M3EN,
                             unsigned char M3DIR,
                             unsigned char M3PWM,
                             unsigned char M3DIAG,
                             unsigned char M3OCM,
                             unsigned char M4EN,
                             unsigned char M4DIR,
                             unsigned char M4PWM,
                             unsigned char M4DIAG,
                             unsigned char M4OCM);

    // User-defined pin selection.
    DualTB9051FTGMotorShieldMod3230(unsigned char M1EN,
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
                             unsigned char M4OCM);

    // PUBLIC METHODS
    void init(); // Initialize pins and timer1 if applicable.
    void setM1Speed(int speed); // Set speed for M1.
    void setM2Speed(int speed); // Set speed for M2.
    void setM3Speed(int speed); // Set speed for M3.
    void setM4Speed(int speed); // Set speed for M4.
    void setSpeeds(int m1Speed, int m2Speed, int m3speed, int m4speed); // Set speed for all Motors.
    unsigned char getM1Fault(); // Get fault reading from M1.
    unsigned char getM2Fault(); // Get fault reading from M2.
    unsigned char getM3Fault(); // Get fault reading from M3.
    unsigned char getM4Fault(); // Get fault reading from M4.
    void flipM1(boolean flip); // Flip the direction of the speed for M1.
    void flipM2(boolean flip); // Flip the direction of the speed for M2.
    void flipM3(boolean flip); // Flip the direction of the speed for M3.
    void flipM4(boolean flip); // Flip the direction of the speed for M4.
    void enableM1Driver(); // Enable the driver for M1.
    void enableM2Driver(); // Enable the driver for M2.
    void enableM3Driver(); // Enable the driver for M3.
    void enableM4Driver(); // Enable the driver for M4.
    void enableDrivers(); // Enables the drivers for all motors.
    void disableM1Driver(); // Disable the driver for M1.
    void disableM2Driver(); // Disable the driver for M2.
    void disableM3Driver(); // Disable the driver for M1.
    void disableM4Driver(); // Disable the driver for M2.
    void disableDrivers(); // Disable the drivers for all motors.
    unsigned int getM1CurrentMilliamps(); // Get current reading for M1.
    unsigned int getM2CurrentMilliamps(); // Get current reading for M2.
    unsigned int getM3CurrentMilliamps(); // Get current reading for M3.
    unsigned int getM4CurrentMilliamps(); // Get current reading for M4.

  private:
    unsigned char _M1PWM;
    static const unsigned char _M1PWM_TIMER1_PIN = 9; //Arduino Uno
    static const unsigned char _M1PWM_TIMER2_PIN = 9; //Arduino Mega
    unsigned char _M2PWM;
    static const unsigned char _M2PWM_TIMER1_PIN = 10;  //Arduino Uno
    static const unsigned char _M2PWM_TIMER2_PIN = 10;  //Arduino Mega
    unsigned char _M3PWM;
    static const unsigned char _M3PWM_TIMER5_PIN = 45;
    unsigned char _M4PWM;
    static const unsigned char _M4PWM_TIMER5_PIN = 46;
    unsigned char _M1EN;
    unsigned char _M2EN;
    unsigned char _M3EN;
    unsigned char _M4EN;
    unsigned char _M1DIR;
    unsigned char _M2DIR;
    unsigned char _M3DIR;
    unsigned char _M4DIR;
    unsigned char _M1DIAG;
    unsigned char _M2DIAG;
    unsigned char _M3DIAG;
    unsigned char _M4DIAG;
    unsigned char _M1OCM;
    unsigned char _M2OCM;
    unsigned char _M3OCM;
    unsigned char _M4OCM;
    boolean _flipM1;
    boolean _flipM2;
    boolean _flipM3;
    boolean _flipM4;
};
