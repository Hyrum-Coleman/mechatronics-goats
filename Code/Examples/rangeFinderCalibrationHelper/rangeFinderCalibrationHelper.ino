#include <Encoder.h>
#include <DualTB9051FTGMotorShieldMod3230.h>

DualTB9051FTGMotorShieldMod3230 wheels;

// Define encoders for each wheel
Encoder gEnc1(18, 22);  // wheel 1 (interrupt pin, non-interrupt pin)
Encoder gEnc2(3, 24);   // wheel 2 (interrupt pin, non-interrupt pin)
Encoder gEnc3(2, 26);   // wheel 3 (interrupt pin, non-interrupt pin)
Encoder gEnc4(19, 28);  // wheel 4 (interrupt pin, non-interrupt pin)

// Variables to store the previous counts for each encoder
int32_t prevCount1 = 0;
int32_t prevCount2 = 0;
int32_t prevCount3 = 0;
int32_t prevCount4 = 0;

unsigned long prevTime = 0;
const int cEncoderCountsPerRev = 64;
const int cWheelMotorGearRatio = 50;
const float cWheelRadius = 1.2795;
const int cDistPin1 = A4;  // Left IR rangefinder sensor
const int cDistPin2 = A5;  // Right IR rangefinder sensor
float distance = 0;
bool done = false;

void setup() {
  Serial2.begin(9600);
  wheels.init();
  wheels.enableDrivers();
  // Initialize previous counts for each encoder
  prevCount1 = gEnc1.read();
  prevCount2 = gEnc2.read();
  prevCount3 = gEnc3.read();
  prevCount4 = gEnc4.read();
  prevTime = millis();
  // drive backwards
  wheels.setSpeeds(-75, 75, -75, 75);
  pinMode(cDistPin1, INPUT);
  pinMode(cDistPin2, INPUT);
  Serial2.println("BEGIN DATA");
}

void loop() {
  if (!done) {
    if (millis() - prevTime >= 100) {
      displayData();
      prevTime = millis();
    }
    if (currentAvgDistanceBackwards() >= 5.5) {
      wheels.setSpeeds(0, 0, 0, 0);
      Serial2.println("END DATA");
      done = true;
    }
  }
}

/**
* Converts an encoder count to the corresponding angular displacement of a wheel.
* 
* @param count : encoder count
* @return theta : wheel displacement in rads
*/
float encoderCountToTheta(float count) {
  // (count * 2pi) / (countsPerRev*gearRatio)
  float theta = (count * 2 * PI) / (cEncoderCountsPerRev * cWheelMotorGearRatio);
  return theta;
}

/**
* Converts an encoder count to the corresponding angular displacement of a wheel.
* 
* @param theta : angular displacement in rads
* @return inches : how far the center of the wheel would move assuming rolling without slipping
*/
float thetaToInches(float theta) {
  // x = theta*r
  float inches = theta * cWheelRadius;
  return inches;
}

float currentAvgDistanceBackwards() {
  // Read current counts for each encoder
  int32_t currCount1 = -gEnc1.read();
  int32_t currCount2 = gEnc2.read();
  int32_t currCount3 = -gEnc3.read();
  int32_t currCount4 = gEnc4.read();
  float count_avg = (currCount1 + currCount2 + currCount3 + currCount4) / 4.0;
  float theta = encoderCountToTheta(count_avg);
  float inches = thetaToInches(theta);
  return -inches;
}

// Displays distance calculated with encoders next to distance reported by distance sensors
void displayData() {
  float inches = currentAvgDistanceBackwards();
  float distanceLeft = analogRead(cDistPin1);
  float distanceRight = analogRead(cDistPin2);

  Serial2.print(inches);
  Serial2.print(",");
  Serial2.print(distanceLeft);
  Serial2.print(",");
  Serial2.print(distanceRight);
  Serial2.println(",");
}
