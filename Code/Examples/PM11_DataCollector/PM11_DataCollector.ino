// Lab11Template.ino
// Prof. Stephen Mascaro
// 11/06/23

//  This sketch will be used to perform line localization and line following
#include <Encoder.h>
#include <DualTB9051FTGMotorShieldMod3230.h>

// Define encoders for each wheel
Encoder gEnc1(18, 22);  // wheel 1 (interrupt pin, non-interrupt pin)
Encoder gEnc2(3, 24);   // wheel 2 (interrupt pin, non-interrupt pin)
Encoder gEnc3(2, 26);   // wheel 3 (interrupt pin, non-interrupt pin)
Encoder gEnc4(19, 28);  // wheel 4 (interrupt pin, non-interrupt pin)

DualTB9051FTGMotorShieldMod3230 gMecanumMotors;

double t, t_old, t0, deltaT, print_time=0; // declare some time variables

long counts1, counts2, counts3, counts4;
int m1command=200, m2command=200;  //declare and initialize motor commands

double GearRatio = 50; 
int countsPerRev = 64; // encoder counts per Rev
//double rw = 4.2; // wheel radius in cm
//double D = 26; // distance between wheels in cm

double theta1, omega1, theta1_old = 0;
double theta2, omega2, theta2_old = 0;
double theta3, omega3, theta3_old = 0;
double theta4, omega4, theta4_old = 0;
//double omega1f=0,omega2f=0, alpha=0.01; //digital filter
//double m1current, m1currentf;
String inString = "";

void setup() {
  // initialize reflectance sensor and motor driver
  gMecanumMotors.init(); 
  gMecanumMotors.enableDrivers();

  Serial.begin(9600);
   
  t0 = micros()/1000000.;
  t_old = 0;
}

void loop() {

  t = micros()/1000000.-t0;
  deltaT = t-t_old;  // sample time

  counts1 = gEnc1.read();
  counts2 = gEnc2.read();
  counts3 = gEnc3.read();
  counts4 = gEnc4.read();

  // calculate your position and velocity here
  theta1 = 2*PI*counts1/(countsPerRev*GearRatio);
  omega1 = (theta1 - theta1_old)/deltaT;

  theta2 = 2*PI*counts2/(countsPerRev*GearRatio);
  omega2 = (theta2 - theta2_old)/deltaT;

  theta3 = 2*PI*counts3/(countsPerRev*GearRatio);
  omega3 = (theta3 - theta3_old)/deltaT;

  theta4 = 2*PI*counts4/(countsPerRev*GearRatio);
  omega4 = (theta4 - theta4_old)/deltaT;

  float omegaAvg = (omega1+omega2+omega3+omega4) / 4.0;
  
  //Put step command here
  bool step = (1 < t && t < 2);
  md.setSpeeds(m1command * step, m2command * step); // send motor commands
  
  // Put print commands here
  Serial.print(t);
  Serial.print("\t");
  Serial.print(omegaAvg);
  Serial.print("\t");
  Serial.print(m1command * step);
  Serial.println("");


  t_old = t;
  theta1_old = theta1;
  theta2_old = theta2;
}
