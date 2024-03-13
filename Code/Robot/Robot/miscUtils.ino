void setPinModes() {
  pinMode(cTopLimitSwitchPin, INPUT);
  pinMode(cBottomLimitSwitchPin, INPUT);
  pinMode(cHallSensorPin, INPUT);
  pinMode(cDistPin1, INPUT);
  pinMode(cDistPin2, INPUT);
}

// Prints all of the sensor values at the current time neatly.
void debugPrintSensors() {
  float hallVoltage = getCurrentHallVoltage();
  RGB colorReading = readGlobalColorSensor();
  int total = colorReading.r + colorReading.g + colorReading.b;
  uint8_t rgbProximity = gApds.readProximity();
  uint16_t linePosition = gQtr.readLineBlack(gLineSensorValues);
  float distanceLeft = pollRangefinder(cDistPin1);
  float distanceRight = pollRangefinder(cDistPin2);
  BlockColor predictedColor = predictColor(colorReading);

  Serial2.print("Hall: ");
  Serial2.print(hallVoltage, 2);

  Serial2.print(" | RGB: (");
  Serial2.print(colorReading.r);
  Serial2.print(",");
  Serial2.print(colorReading.g);
  Serial2.print(",");
  Serial2.print(colorReading.b);
  Serial2.print(") = (");
  Serial2.print((float)colorReading.r / total, 2);
  Serial2.print(",");
  Serial2.print((float)colorReading.g / total, 2);
  Serial2.print(",");
  Serial2.print((float)colorReading.b / total, 2);
  Serial2.print(")");

  Serial2.print(" | apdsProx: ");
  Serial2.print(rgbProximity);

  Serial2.print(" | Line: ");
  Serial2.print(linePosition);

  Serial2.print(" | Prox (L,R): (");
  Serial2.print(distanceLeft, 2);
  Serial2.print(",");
  Serial2.print(distanceRight, 2);
  Serial2.print(")");

  Serial2.print(" | Color: ");
  Serial2.print(blockColorToString(predictedColor));

  if (hallVoltage > cHallReloadingThreshold) {
    Serial2.print(" | Magnet: Yes");
  } else {
    Serial2.print(" | Magnet: No");
  }

  Serial2.println("");

  delay(200);
}
