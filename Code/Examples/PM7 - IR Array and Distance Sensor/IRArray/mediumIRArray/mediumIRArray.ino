#include <QTRSensors.h>
#include <DualTB9051FTGMotorShieldMod3230.h>

QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
String message = "";

DualTB9051FTGMotorShieldMod3230 wheels;

void setup() {
  Serial2.begin(9600);
  Serial2.println("Connected!");

  // Initialize line following sensors.
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]) {
    36, 38, 40, 42, 43, 41, 39, 37
  }, SensorCount);

  // Calibrate for 3s -- MOVE THE ROBOT AROUND THE LINE DURING THIS TIME.
  for (uint16_t i = 0; i < 100; i++) {
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
    double Kp = 1.0 / 10.0;

    int baseSpeed = 300; // Base speed for each motor
    int leftSpeed = baseSpeed + (Kp * error); // YOU MAY NEED TO FLIP THE + AND - HERE, IM GUESSING!!!!
    int rightSpeed = baseSpeed - (Kp * error);

    // Set the speeds -- tank driving style
    wheels.setSpeeds(leftSpeed, -rightSpeed, leftSpeed, -rightSpeed);

    delay(100); // Short delay to avoid overly frequent adjustments
  }
}
