// Lab7Template.ino
// Prof. Stephen Mascaro
// 10/05/23

//  This sketch will be used to perform line localization and line following
#include <QTRSensors.h>
#include <DualTB9051FTGMotorShieldUnoMega.h>

QTRSensors qtr;                      // create a reflectance sensor object
DualTB9051FTGMotorShieldUnoMega md;  // create a motor driver object

const uint8_t SensorCount = 8;       // # of sensors in reflectance array
uint16_t sensorValues[SensorCount];  //reflectance sensor readings
uint16_t sensorValuesUnbiased[SensorCount];
double t, t0, print_time = 0;        // declare some time variables

double Kp = 1;            //Proportional Gain for Line Following
double base_speed = 100;  //Nominal speed of robot

int m1c = 0, m2c = 0;  //declare and initialize motor commands

void setup() {
  // initialize reflectance sensor and motor driver
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){ 27, 29, 31, 33, 35, 37, 39, 41 }, SensorCount);
  md.init();
  md.enableDrivers();

  Serial.begin(9600);

  // Uncomment the following code in order to input Kp and base_speed from Serial Monitor
  String inString = "";
  Serial.println("Input values for Kp and base_speed with a space between");
  while(Serial.available()==0);
  inString = Serial.readStringUntil(' ');
  Kp = inString.toFloat();
  inString = Serial.readStringUntil(' ');
  base_speed = inString.toFloat();
  
  t0 = micros()/1000000.;
  Serial.print("Kp = ");
  Serial.println(Kp);
  Serial.print("Base Speed = ");
  Serial.println(base_speed);

  t0 = micros() / 1000000.;  // initialize time
}

void loop() {
  static uint16_t sensor_bias[] = { 92, 140, 92, 140, 140, 140, 92, 140 };
  static uint16_t sensor_position[] = {0, .8, 1.6, 2.4, 3.2, 4, 4.8, 5.6};

  t = micros() / 1000000. - t0;
  qtr.read(sensorValues);  // sensor values will be numbers between 0 and 2500

  // put your line localization code here

  // put your line following code here

  for (int i = 0; i <  SensorCount; i++) {
    sensorValuesUnbiased[i] = sensorValues[i] - sensor_bias[i];
  }


  float numeratorSum = 0;
  float denominatorSum = 0;
  for (int i = 0; i < SensorCount; i++) {
    numeratorSum += sensorValuesUnbiased[i] * sensor_position[i];
    denominatorSum += sensorValuesUnbiased[i];
  }

  float d = numeratorSum / denominatorSum;
  float error = 2 - d;

  m1c = base_speed - Kp * error;
  m2c = base_speed + Kp * error;

  md.setSpeeds(m1c, m2c);  // send motor commands

  if ((t - print_time) > 0.25) {  // non-blocking way to delay printing
    // print any variables of interest inside this if statement
    for (uint8_t i = 0; i < SensorCount; i++) {
      Serial.print(sensorValues[i]);
      Serial.print('\t');
    }
    Serial.println(d);
    Serial.println(error);
    // Serial.println(d);
    print_time = t;
  }
}
