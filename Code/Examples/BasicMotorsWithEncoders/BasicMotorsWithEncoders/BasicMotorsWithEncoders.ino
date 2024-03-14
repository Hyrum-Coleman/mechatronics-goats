#include <L298NMotorDriverMega.h>
#include <DualTB9051FTGMotorShieldMod3230.h>
#include <Encoder.h>

// Motor driver setup
L298NMotorDriverMega l2_driver(5, 34, 32, 6, 33, 35);
DualTB9051FTGMotorShieldMod3230 wheels;

// Encoder setup
Encoder gEnc1(18, 22); // wheel 1 (interrupt pin, non-interrupt pin)
Encoder gEnc2(3, 24);  // wheel 2 (interrupt pin, non-interrupt pin)
Encoder gEnc3(2, 26);  // wheel 3 (interrupt pin, non-interrupt pin)
Encoder gEnc4(19, 28); // wheel 4 (interrupt pin, non-interrupt pin)

String message = "";
unsigned long previousMillis = 0; // will store last time the motors were updated
unsigned long encoderPrintPreviousMillis = 0; // will store last time encoder counts were printed
const long interval = 500; // interval at which to run motor commands (milliseconds)
const long encoderPrintInterval = 100; // interval at which to print encoder counts (milliseconds)
bool isMoving = false; // track whether the motors are moving
int movementStage = 0; // to cycle through different movement commands

long encoderPositions[4] = {0, 0, 0, 0}; // to store encoder counts
long previousEncoderPositions[4] = {0, 0, 0, 0}; // to store previous encoder counts

void setup() {
  l2_driver.init();
  wheels.init();
  wheels.enableDrivers();
  Serial2.begin(9600);
}

void loop() {
  if (Serial2.available() > 2) {
    message = Serial2.readStringUntil('\n');
    Serial2.println(message);
  }

  if (message.equals("Go!")) {
    unsigned long currentMillis = millis();

    if (!isMoving) {
      previousMillis = currentMillis; // reset the timer
      isMoving = true; // start moving
      movementStage = 1; // start from the first movement
    }

    if (isMoving && currentMillis - previousMillis >= interval) {
      switch (movementStage) {
        case 1:
          wheels.setSpeeds(255, -255, 255, -255);
          break;
        case 2:
          wheels.setSpeeds(0, 0, 0, 0);
          break;
        case 3:
          wheels.setSpeeds(-255, 255, -255, 255);
          break;
        case 4:
          wheels.setSpeeds(0, 0, 0, 0);
          break;
        case 5:
          wheels.setSpeeds(255, 255, -255, -255);
          break;
        case 6:
          wheels.setSpeeds(0, 0, 0, 0);
          break;
        case 7:
          wheels.setSpeeds(-255, -255, 255, 255);
          break;
        case 8:
          wheels.setSpeeds(0, 0, 0, 0);
          break;
        case 9:
          wheels.setSpeeds(255, 255, 255, 255);
          break;
        case 10:
          wheels.setSpeeds(0, 0, 0, 0);
          break;
        case 11:
          wheels.setSpeeds(-255, -255, -255, -255);
          break;
        case 12:
          wheels.setSpeeds(0, 0, 0, 0);
          // Reset for next command
          isMoving = false;
          message = ""; // Reset the message so it doesn't trigger the movement again
          break;
      }
      movementStage++;
      if (movementStage > 12) movementStage = 1; // Loop back after completing the movements
      previousMillis = currentMillis; // Save the time of the last update
    }
  }

  // Print encoder counts per second
  unsigned long currentMillis = millis();
  if (currentMillis - encoderPrintPreviousMillis >= encoderPrintInterval) {
    encoderPrintPreviousMillis = currentMillis;

    // Read current encoder positions
    encoderPositions[0] = gEnc1.read();
    encoderPositions[1] = gEnc2.read();
    encoderPositions[2] = gEnc3.read();
    encoderPositions[3] = gEnc4.read();

    // Print counts per second for each wheel
    Serial2.print("Encoder Counts per Second - Wheel 1: ");
    Serial2.print((encoderPositions[0] - previousEncoderPositions[0]) * 1000 / encoderPrintInterval);
    Serial2.print(", Wheel 2: ");
    Serial2.print((encoderPositions[1] - previousEncoderPositions[1]) * 1000 / encoderPrintInterval);
    Serial2.print(", Wheel 3: ");
    Serial2.print((encoderPositions[2] - previousEncoderPositions[2]) * 1000 / encoderPrintInterval);
    Serial2.print(", Wheel 4: ");
    Serial2.println((encoderPositions[3] - previousEncoderPositions[3]) * 1000 / encoderPrintInterval);

    // Update previous positions for next calculation
    for (int i = 0; i < 4; i++) {
      previousEncoderPositions[i] = encoderPositions[i];
    }
  }
}
