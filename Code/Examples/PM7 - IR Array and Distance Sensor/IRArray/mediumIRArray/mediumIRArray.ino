#include <QTRSensors.h>
#include <DualTB9051FTGMotorShieldMod3230.h>

QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
String message = "";

DualTB9051FTGMotorShieldMod3230 wheels;

void setup() {
  Serial2.begin(9600);

  // Initialize line following sensors.
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]) {
    36, 37, 38, 39, 40, 41, 42, 43
  }, SensorCount);

  // Calibrate for 10s -- MOVE THE ROBOT AROUND THE LINE DURING THIS TIME.
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }

  // Initialize motor driver
  wheels.init();
  wheels.enableDrivers();
}

void loop() {
  if (Serial2.available() > 2) {
    message = Serial2.readStringUntil('\n');
  }
  if (message.equals("Go!")) {
    uint16_t position = qtr.readLineBlack(sensorValues); // Read the line position
    // Calculate the error from the center.
    // The range is 0 to 7000 ( because 8 sensors). Max is (_sensorCount - 1) * 1000.
    // Hence, 3500 is the middle.
    int error = position - 3500;
    double Kp = 1.0 / 20.0;

    int baseSpeed = 200; // Base speed for each motor
    int leftSpeed = baseSpeed - (Kp * error); // YOU MAY NEED TO FLIP THE + AND - HERE, IM GUESSING!!!!
    int rightSpeed = baseSpeed + (Kp * error);

    // Set the speeds -- tank driving style
    wheels.setSpeeds(leftSpeed, -rightSpeed, leftSpeed, -rightSpeed);

    delay(100); // Short delay to avoid overly frequent adjustments
  }
}
