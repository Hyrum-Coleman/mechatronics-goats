void addBlockToBelt(std::stack<Block>* blocks, Block blockToAdd) {
  blocks->push(blockToAdd);
  return;
}

Block getNextBlock(std::stack<Block>* blocks) {
  Block topBlock = blocks->top();
  blocks->pop();
  return topBlock;
}

Block createBlock(RGB rgb) {
  Block newBlock;
  BlockColor color = predictColor(rgb);

  switch (color) {
    case BlockColor::Red:
      DEBUG_PRINTLN("Red block detected");
      newBlock.color = BlockColor::Red;
      break;
    case BlockColor::Yellow:
      DEBUG_PRINTLN("Yellow block detected");
      newBlock.color = BlockColor::Yellow;
      break;
    case BlockColor::Blue:
      DEBUG_PRINTLN("Blue block detected");
      newBlock.color = BlockColor::Blue;
      break;
    case BlockColor::None:
    default:
      DEBUG_PRINTLN("Uncertain about the color");
      DEBUG_PRINT("RGB: (");
      DEBUG_PRINT(rgb.r);
      DEBUG_PRINT(", ");
      DEBUG_PRINT(rgb.g);
      DEBUG_PRINT(", ");
      DEBUG_PRINT(rgb.b);
      DEBUG_PRINTLN(")");
      DEBUG_PRINTLN("Setting color to None to avoid crashing");
      newBlock.color = BlockColor::None;
  }

  return newBlock;
}

void addBlockToStackFromRGB(std::stack<Block>* blocks, RGB rgb) {
  Block newBlock = createBlock(rgb);

  addBlockToBelt(blocks, newBlock);
}

// Takes a BlockColor enumeration and returns a string that can be printed to the serial monitor. Not used for control flow purposes.
const char* blockColorToString(BlockColor color) {
  switch (color) {
    case BlockColor::Red: return "Red";
    case BlockColor::Yellow: return "Yellow";
    case BlockColor::Blue: return "Blue";
    case BlockColor::None: return "None";
    case BlockColor::UnCalibrated: return "Unc";
    default: return "Unknown";
  }
}

// Predicts the color of a block based on the color reading by comparing it to our calibration values.
BlockColor predictColor(RGB colorReading) {

  if (!isColorSensorCalibrated()) {
    return BlockColor::UnCalibrated;
  }

  if (gApds.readProximity() < 10) {
    return BlockColor::None;
  }

  float threshold = 0.15;

  float colorSample[3] = { float(colorReading.r) / (colorReading.r + colorReading.g + colorReading.b),
                           float(colorReading.g) / (colorReading.r + colorReading.g + colorReading.b),
                           float(colorReading.b) / (colorReading.r + colorReading.g + colorReading.b) };

  // calculate euclidian color distances to each average color reading
  float distanceToRed = colorDistance(averageRedReadings, colorSample);
  float distanceToYellow = colorDistance(averageYellowReadings, colorSample);
  float distanceToBlue = colorDistance(averageBlueReadings, colorSample);

  if (distanceToRed > threshold && distanceToYellow > threshold && distanceToBlue > threshold) {
    return BlockColor::None;  // none of the colors are close enough
  }

  // determine the closest color
  if (distanceToRed <= distanceToYellow && distanceToRed <= distanceToBlue) {
    return BlockColor::Red;
  } else if (distanceToYellow <= distanceToRed && distanceToYellow <= distanceToBlue) {
    return BlockColor::Yellow;
  } else {
    return BlockColor::Blue;
  }
}

// Function to calibrate our color sensor to the three blocks. It's automated/guided.
void calibrateColorSensor() {
  DEBUG_PRINTLN("Calibrate color sensor! First, place the red block.");
  // for each red, yellow, blue blocks: wait for block to be placed infont of sensor.
  for (int i = 0; i < 3; i++) {

    while (gApds.readProximity() <= 10) {
      // Wait for a block to appear
      delay(100);  // Check every 100 milliseconds
    }
    DEBUG_PRINTLN("Block detected. Calculating...");

    // block appears
    // collect 20 samples of normalized color data and average it.
    int sumR = 0;
    int sumG = 0;
    int sumB = 0;

    // collect 20 samples
    for (int j = 0; j < 20; j++) {
      while (!gApds.colorDataReady()) {
        delay(10);
      }
      RGB colorReading = readGlobalColorSensor();
      sumR += colorReading.r;
      sumG += colorReading.g;
      sumB += colorReading.b;
      delay(50);  // small delay between readings
    }

    // calculate average
    float avgR = sumR / 20.0;
    float avgG = sumG / 20.0;
    float avgB = sumB / 20.0;

    // normalize the averaged values
    float total = avgR + avgG + avgB;
    float r_norm = avgR / total;
    float g_norm = avgG / total;
    float b_norm = avgB / total;

    if (i == 0) {  // red block
      DEBUG_PRINTLN("Red block calibrated. Remove red block and place yellow block.");
      averageRedReadings[0] = r_norm;
      averageRedReadings[1] = g_norm;
      averageRedReadings[2] = b_norm;
    } else if (i == 1) {  // yellow block
      DEBUG_PRINTLN("Yellow block calibrated. Remove yellow block and place blue block.");
      averageYellowReadings[0] = r_norm;
      averageYellowReadings[1] = g_norm;
      averageYellowReadings[2] = b_norm;
    } else if (i == 2) {  // blue block
      DEBUG_PRINTLN("Blue block calibrated. All blocks calibrated.");
      averageBlueReadings[0] = r_norm;
      averageBlueReadings[1] = g_norm;
      averageBlueReadings[2] = b_norm;
    }

    // wait for the block to be removed
    while (gApds.readProximity() > 10) {
      delay(100);  // check every 100 milliseconds until the block is removed
    }
    DEBUG_PRINTLN("Block removed.");
  }
}

// Function to calculate the Euclidean distance between two colors. We use it to compare colors similarity. It is the best way to do this.
float colorDistance(float color1[3], float color2[3]) {
  return sqrt(pow(color1[0] - color2[0], 2) + pow(color1[1] - color2[1], 2) + pow(color1[2] - color2[2], 2));
}

// Helper function to check if the sensor is calibrated
bool isColorSensorCalibrated() {
  // check if any of the average readings arrays still has its initial value
  for (int i = 0; i < 3; i++) {
    if (averageRedReadings[i] == -1 || averageYellowReadings[i] == -1 || averageBlueReadings[i] == -1) {
      return false;  // not calibrated
    }
  }
  return true;  // calibrated
}

// This function reads the color sensor and stores it in the RGB struct
// Important to note that the clear channel value is currently being discarded.
RGB readGlobalColorSensor() {
  if (!gApds.colorDataReady()) {
    DEBUG_PRINTLN("Failed to collect color data");
    return RGB();
  }

  RGB rgb;
  uint16_t c;

  gApds.getColorData(&rgb.r, &rgb.g, &rgb.b, &c);

  return rgb;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

// Calibrates the IR array by spinning in a circle while calling the calibrate method of the QTR library (which asks you to move the array over a line)
void calibrateIrArray(Move nextMove) {
  // input 'nextMove' is not yet used. In future, it will have associated calibration types. For now, just calibrate everything.
  // 10s is 400, so 1s is 40
  int duration = nextMove.params.calibrationParams.duration / 1000;  // convert to s
  DEBUG_PRINTLN(duration);
  gMecanumMotors.setSpeeds(200, 200, 200, 200);
  for (uint16_t i = 0; i < 40 * duration; i++) {
    gQtr.calibrate();
  }
  gMecanumMotors.setSpeeds(0, 0, 0, 0);
  DEBUG_PRINTLN("Done calibrating.");
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

// Function to calculate distance based on sensor reading for the left sensor
double calculateDistanceLeft(int sensorValue, DistanceCalibrationMaterial mat) {
  // This catches low noise that blows up our value
  if (sensorValue < 75) {  // min effectiveness of our sensor
    return 5.5;            // Max distance we can see
  }

  double a;
  double b;

  switch (mat) {
    case DistanceCalibrationMaterial::Button:
      a = 0; // NOT YET CALIBRATED
      b = 0; // NOT YET CALIBRATED
      break;
    case DistanceCalibrationMaterial::Chassis:
      a = 0; // NOT YET CALIBRATED
      b = 0; // NOT YET CALIBRATED
      break;
    default:
      a = -0.73810816;
      b = 5.55203935;
      break;
  }

  double logSensorValue = log(sensorValue);
  double distance = exp((logSensorValue - b) / a);
  return distance;
}

// Function to calculate distance based on sensor reading for the right sensor
double calculateDistanceRight(int sensorValue, DistanceCalibrationMaterial mat) {
  // This catches low noise that blows up our value
  if (sensorValue < 75) {  // min effectiveness of our sensor
    return 5.5;            // Max distance we can see
  }

  double a;
  double b;

  switch (mat) {
    case DistanceCalibrationMaterial::Button:
      a = 0; // NOT YET CALIBRATED
      b = 0; // NOT YET CALIBRATED
      break;
    case DistanceCalibrationMaterial::Chassis:
      a = 0; // NOT YET CALIBRATED
      b = 0; // NOT YET CALIBRATED
      break;
    default:
      a = -0.6871539;
      b = 5.5796678;
      break;
  }

  double logSensorValue = log(sensorValue);
  double distance = exp((logSensorValue - b) / a);
  return distance;
}

// Returns the calculated distance of our rangefinder.
float getRangeFinderRawReading(int pin) {
  int sensorValue = analogRead(pin);
  return sensorValue;
}

// The next two functions are pretty similar. I'm still trying to think of a better way.
// Returns the calculated distance of our rangefinder.
float getDistFromRangeFinder(int pin, DistanceCalibrationMaterial mat) {
  int sensorValue = getRangeFinderRawReading(pin);

  if (pin == cDistPin1) {
    return calculateDistanceLeft(sensorValue, mat);
  }
  if (pin == cDistPin2) {
    return calculateDistanceRight(sensorValue, mat);
  }
  // else? idk. just dont call it with the wrong pin ig. skill issue if you do.
}

// Returns the calculated distance of our rangefinder. Now uses an IIR filter
float getDistFromRangeFinderFiltered(int pin, DistanceCalibrationMaterial mat) {

  int sensorValue = getRangeFinderRawReading(pin);
  float distance;

  if (pin == cDistPin1) {
    distance = calculateDistanceLeft(sensorValue, mat);
    return gDistLeftFilter.process(distance);
  }
  if (pin == cDistPin2) {
    distance = calculateDistanceRight(sensorValue, mat);
    return gDistRightFilter.process(distance);
  }
  // else? idk. just dont call it with the wrong pin ig. skill issue if you do.
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

// Prints all of the sensor values at the current time neatly.
void debugPrintSensors() {
  float hallVoltage = getCurrentHallVoltage();
  float hallVoltageFiltered = getCurrentHallVoltageFiltered();
  RGB colorReading = readGlobalColorSensor();
  int total = colorReading.r + colorReading.g + colorReading.b;
  uint8_t rgbProximity = gApds.readProximity();
  uint16_t linePosition = gQtr.readLineBlack(gLineSensorValues);
  float distanceLeft = getDistFromRangeFinder(cDistPin1, DistanceCalibrationMaterial::Cardboard);
  float distanceRight = getDistFromRangeFinder(cDistPin2, DistanceCalibrationMaterial::Cardboard);
  float distanceLeftFiltered = getDistFromRangeFinderFiltered(cDistPin1, DistanceCalibrationMaterial::Cardboard);
  float distanceRightFiltered = getDistFromRangeFinderFiltered(cDistPin2, DistanceCalibrationMaterial::Cardboard);
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

  if (isMagnetDetected()) {
    Serial2.print(" | Magnet: Yes");
  } else {
    Serial2.print(" | Magnet: No");
  }

  Serial2.print(" | ");

  Serial2.print("Hall(IIR): ");
  Serial2.print(hallVoltageFiltered, 2);

  Serial2.print(" | Prox IIR (L,R): (");
  Serial2.print(distanceLeftFiltered, 2);  // this appears slow because we aren't polling very fast in this mode!
  Serial2.print(",");
  Serial2.print(distanceRightFiltered, 2);
  Serial2.print(")");

  Serial2.println("");

  delay(200);
}

bool isMagnetDetected() {
  return abs(getCurrentHallVoltageFiltered() - cHallReloadingQuiescent) > 50;
}

// Returns the voltage of the hall effect sensor
float getCurrentHallVoltage() {
  float hallVoltage = analogRead(cHallSensorPin);
  return hallVoltage;
}

float getCurrentHallVoltageFiltered() {
  float hallVoltage = analogRead(cHallSensorPin);
  return gHallFilter.process(hallVoltage);
}
