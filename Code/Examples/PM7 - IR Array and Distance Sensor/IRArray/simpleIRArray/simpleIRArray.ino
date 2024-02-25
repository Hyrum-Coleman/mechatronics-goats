#include <QTRSensors.h>

// This example is designed for use with eight RC QTR sensors. These
// reflectance sensors should be connected to digital pins 3 to 10. The
// sensors' emitter control pin (CTRL or LEDON) can optionally be connected to
// digital pin 2, or you can leave it disconnected and remove the call to
// setEmitterPin().
//
// The main loop of the example reads the raw sensor values (uncalibrated). You
// can test this by taping a piece of 3/4" black electrical tape to a piece of
// white paper and sliding the sensor across it. It prints the sensor values to
// the serial monitor as numbers from 0 (maximum reflectance) to 2500 (minimum
// reflectance; this is the default RC timeout, which can be changed with
// setTimeout()).

QTRSensors qtr;
String message = "";

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

void setup()
{
  Serial2.begin(9600);
  // CONFIGURE
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]) {
    36, 38, 40, 42, 43, 41, 39, 37
  }, SensorCount);

  // BEGIN CALIBRATION -- MOVE AROUND OVER BLACK LINE FOR ~10s TO CALIBRATE.
  // 400 calibrations takes about 10s.
  // BUILTIN LED TURNS OFF WHEN DONE CALIBRATING.
  // ----------------------------------------------------------------------
  Serial2.println("Calibrating -- move array over black line for ~10s. Light turns off when done.");
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }

  digitalWrite(LED_BUILTIN, LOW);
  // ------------------- PRINT CALIBRATION RESULTS ------------------------
  Serial2.println("Calibration Results:");
  Serial2.println("Minimum Reflectance (Calibration On):");
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial2.print("Sensor ");
    Serial2.print(i + 1);
    Serial2.print(": ");
    Serial2.print(qtr.calibrationOn.minimum[i]);
    Serial2.println();
  }

  Serial2.println("\nMaximum Reflectance (Calibration On):");
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial2.print("Sensor ");
    Serial2.print(i + 1);
    Serial2.print(": ");
    Serial2.print(qtr.calibrationOn.maximum[i]);
    Serial2.println();
  }
  Serial2.println(); // Extra line for spacing
  delay(1000); // Delay to ensure readability
  // ------------------- END CALIBRATION --------------------------------

  Serial2.println("Calibrated. Waiting for start signal (from XBee) before printing values.");
}

void loop()
{
  if (Serial2.available() > 2) {
    message = Serial2.readStringUntil('\n');
  }
  if (message.equals("Go!")) {
    // read calibrated sensor values and obtain a measure of the line position
    // position is from 0 to 7000 -- 0 is under sensor 1, 7000 is under sensor 8
    uint16_t position = qtr.readLineBlack(sensorValues);

    // Print a header for the sensor values for clarity
    Serial2.print("Sensor Values: ");
    for (uint8_t i = 0; i < SensorCount; i++)
    {
      Serial2.print("S");
      Serial2.print(i + 1); // Sensor number
      Serial2.print(": ");
      Serial2.print(sensorValues[i]);
      Serial2.print("\t"); // Tab space for separation
    }

    // Print the line position with a label
    Serial2.print("Line Position: ");
    Serial2.println(position);

    delay(250); // Delay for readability=
  }
}
