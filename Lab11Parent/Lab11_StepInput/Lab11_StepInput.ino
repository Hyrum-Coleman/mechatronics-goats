// Lab11Template.ino
// Prof. Stephen Mascaro
// 11/06/23

//  This sketch will be used to perform line localization and line following
#include <Encoder.h>
#include <DualTB9051FTGMotorShieldUnoMega.h>

Encoder myEnc1(21, 20);  // create an encoder object
Encoder myEnc2(18, 19);

DualTB9051FTGMotorShieldUnoMega md;  // create a motor driver object

double t, t_old, t0, deltaT, print_time = 0;  // declare some time variables

long counts1, counts2;
int m1command = 0, m2command = 0;  //declare and initialize motor commands

double GearRatio = 70;
int countsPerRev = 64;  // encoder counts per Rev
double rw = 4.2;        // wheel radius in cm
double D = 26;          // distance between wheels in cm

double theta1, omega1, theta1_old = 0;
double theta2, omega2, theta2_old = 0;
double omega1f = 0, omega2f = 0, alpha = 0.01;  //digital filter
double m1current, m1currentf;
String inString = "";

void setup() {
  // initialize reflectance sensor and motor driver
  md.init();
  md.enableDrivers();

  Serial.begin(9600);

  t0 = micros() / 1000000.;
  t_old = 0;
}

void loop() {

  t = micros() / 1000000. - t0;
  deltaT = t - t_old;  // sample time

  bool timeComparison = (((t > 0) && t < 1) || (t > 2));
  m1command = timeComparison ? 0 : 200;
  m2command = timeComparison ? 0 : 200;



  counts1 = myEnc1.read();
  //  m1current = md.getM1CurrentMilliamps()/1000.; //current in Amps
  //  m1currentf = alpha*m1current + (1-alpha)*m1currentf; //filtered current

  // calculate your position and velocity here
  theta1 = 2 * PI * counts1 / (countsPerRev * GearRatio);
  omega1 = (theta1 - theta1_old) / deltaT;

  theta2 = 2 * PI * counts2 / (countsPerRev * GearRatio);
  omega2 = (theta2 - theta2_old) / deltaT;

  //Put step command here


  md.setSpeeds(m1command, m2command);  // send motor commands

  // Put print commands here
  Serial.print("\t");  //maybe unnecessary
  Serial.print(t);
  Serial.print("\t");
  Serial.print(omega1);
  Serial.print("\t");
  Serial.println(m1command);

  t_old = t;
  theta1_old = theta1;
}
