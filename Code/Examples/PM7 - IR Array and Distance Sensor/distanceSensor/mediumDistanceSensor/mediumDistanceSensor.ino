// Drives up to the wall, stops, then rotates until square with wall....
// #mecanum flex

#include <DualTB9051FTGMotorShieldMod3230.h>

// Define the pins for the distance sensors
const int distPin1 = A4; // Left sensor
const int distPin2 = A5; // Right sensor

DualTB9051FTGMotorShieldMod3230 wheels;
String message = "";

// Target distance from the wall in cm
float targetDistance = 10.0;

void setup() {
  Serial2.begin(9600);
  wheels.init();
  wheels.enableDrivers();
}

void loop() {
  if (Serial2.available() > 2) {
    message = Serial2.readStringUntil('\n');
  }

  if (message.equals("Go!")) {
    float distanceLeft = measureDistance(distPin1);
    float distanceRight = measureDistance(distPin2);

    // Approach the wall
    if (distanceLeft > targetDistance || distanceRight > targetDistance) {
      // Drive forward
      wheels.setSpeeds(255, -255, 255, -255);
    } else {
      // Stop
      wheels.setSpeeds(0, 0, 0, 0);
      // Square up to wall
      rotateToEqualDistance();
    }

    delay(100); // Delay to prevent too frequent updates
  }
}

void rotateToEqualDistance() {
  float distanceLeft = measureDistance(distPin1);
  float distanceRight = measureDistance(distPin2);
  float tolerance = 0.2;

  // Loop to adjust orientation until the robot is squared with the wall
  while (abs(distanceRight - distanceLeft) > tolerance) {
    if (distanceLeft < distanceRight) {
      wheels.setSpeeds(-100, -100, -100, -100);
    } else {
      wheels.setSpeeds(100, 100, 100, 100);
    }

    // Delay briefly to allow the rotation to take effect before remeasuring
    delay(100);

    // Update distances after adjustment
    distanceLeft = measureDistance(distPin1);
    distanceRight = measureDistance(distPin2);
  }

  // Stop all wheels once squared up with the wall
  wheels.setSpeeds(0, 0, 0, 0);
}


float measureDistance(int pin) {
  int sensorValue = analogRead(pin);
  float voltage = sensorValue * (5.0 / 1023.0);
  return calculateDistance(voltage);
}

float calculateDistance(float voltage) {
  // Polynomial conversion from voltage to distance
  float distance = 33.9 - 69.5 * voltage + 62.3 * pow(voltage, 2) - 25.4 * pow(voltage, 3) + 3.83 * pow(voltage, 4);
  return distance;
}
