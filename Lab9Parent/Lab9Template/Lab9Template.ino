// Lab9Template.ino
// Prof. Stephen Mascaro
// 10/23/23

//  This sketch will be used to perform line localization and line following
#include <Encoder.h>
#include <DualTB9051FTGMotorShieldUnoMega.h>

Encoder myEnc1(21, 20);  // create an encoder object
Encoder myEnc2(19, 18);
// Encoder myEnc2(3, 67);
// Encoder myEnc3(66, 67);

DualTB9051FTGMotorShieldUnoMega md;  // create a motor driver object

double t, t_old, deltaT, print_time, t0 = 0;  // declare some time variables

double Kp = 10;  //Proportional Gain for Trajectory Following

long counts1, counts2, counts3;  // encoder counts
int m1c = 0, m2c = 0;            //declare and initialize motor commands

double GearRatio = 70;  // gear ratio
int countsPerRev = 64;  // encoder counts per Rev
double rw = 4.2;        // wheel radius in cm
double D = 26;          // distance between wheels in cm

double theta1, theta1_old = 0, omega1;  //position and velocity of wheel 1
double theta2, theta2_old = 0, omega2;  //position and velocity of wheel 2
double omega1f = 0, alpha = .4;
double omega2f = 0;                        // filtered velocity and filter weight
double theta1_des = 0, theta2_des = 0;     // desired position of wheels
double theta1_final, theta2_final;         // final desired position of wheels
double omega_des, omega1_des, omega2_des;  // desired velocity of wheels
double V1m, V2m;

void setup() {
  // initialize reflectance sensor and motor driver
  md.init();
  md.enableDrivers();

  Serial.begin(9600);
  t_old = micros() / 1000000.;
  t0 = micros() / 1000000.;

  theta1_final = ((3.1415 / 2) * 30) / rw;
  theta2_final = ((3.1415 / 2) * (30 + D)) / rw;
  omega_des = 2.5;
  omega1_des = theta1_final / 2;
  omega2_des = theta2_final / 2;
}

void loop() {

  t = micros() / 1000000. - t0;
  deltaT = t - t_old;  // sample time

  counts1 = myEnc1.read();
  counts2 = myEnc2.read();
  // counts3 = myEnc3.read();

  // calculate your position and velocity here
  theta1_old = theta1;
  theta1 = (counts1) / (GearRatio * countsPerRev) * 2 * 3.1415;

  theta2_old = theta2;
  theta2 = (-counts2) / (GearRatio * countsPerRev) * 2 * 3.1415;

  omega1 = (theta1 - theta1_old) / deltaT;
  omega2 = (theta2 - theta2_old) / deltaT;

  omega1f = alpha * omega1 + (1 - alpha) * omega1f;
  omega2f = alpha * omega2 + (1 - alpha) * omega1f;



  // add your trajectory design here

  theta1_des += omega1_des * deltaT;
  theta2_des += omega2_des * deltaT;

  V1m = Kp * (theta1_des - theta1);
    

  if (abs(theta1_final) <= abs(theta1_des)) {
    theta1_des = theta1_final;
    
  }

  if (abs(theta2_final) <= abs(theta2_des)) {
    theta2_des = theta2_final;
  }


  // add your control laws here



  // Uncomment these four lines in section 4.4
  V1m = constrain(V1m,-10,10);
  V2m = constrain(V2m,-10,10);
  m1c = 400*V1m/10;
  m2c = 400*V2m/10;

  md.setSpeeds(m1c, m2c);  // send motor commands



  if ((t - print_time) > 0) {  // non-blocking way to delay printing
    // print any variables of interest inside this if statement
    // Serial.print(counts1);
    // Serial.print('\t');
    // Serial.print(counts2);
    // Serial.print('\t');
    // Serial.print(counts3);
    // Serial.print('\t');
        Serial.print(theta1_des);
    Serial.print('\t');
    Serial.print(theta1_final);
     Serial.print('\t');
    Serial.print(theta2_des);
    Serial.print('\t');
    Serial.print(theta2_final);
    Serial.println('\t');


    print_time = t;
  }

  t_old = t;
  theta1_old = theta1;
}
