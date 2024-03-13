// Does the whole reload sequence.
// At start: robot is near the reloader over the line.
// At end: robot is near the reloader over the line, but now has a full platform of blocks.
// TODO: need to add platform height logic. Cant yet because no limit switches yet.
void executeReload(std::stack<Block>* blocks) {
  // Drive belt backwards to collect blocks as they enter the belt.
  // In future, make the belt only drive when we need it to. I just dont know the timings yet.
  bool linedUp = false;
  gL2Motors.setM1Speed(-400);
  // While our belt is not full of blocks,
  while (blocks->size() < cMaxBlocks) {
    if (!linedUp) {
      DEBUG_PRINTLN("SQUARING UP");
      squareUpUsingProx(70);
      delay(500);

      DEBUG_PRINTLN("CENTERING");
      centerOnIrArray(70);
      delay(500);

      DEBUG_PRINTLN("SQUARING UP AGAIN");
      squareUpUsingProx(70);

      linedUp = true;
    }

    // Uncomment this when rest of reloading works
    // If the other team has pushed the button, we should wait until its ready to be pushed (using hall effect sensor)
    if (getCurrentHallVoltage() < cHallReloadingThreshold) {  // check < vs > here
      // if the magnet is not detected, the platform is up too high, meaning it is not yet ready for reloading.
      // in that instance, we skip this iteration of the loop and wait until it is detected.
      continue;
    }
    DEBUG_PRINTLN("PLATFORM READY. PUSHING BUTTON.");
    pushButton(70);

    linedUp = false;

    // When block is in front of color sensor/proximity sensor, detect its color and save it to the block stack
    unsigned long startTime = millis();
    bool blockDetected = false;
    // Wait until the block passes in front of the color sensor. Times out after some amount of time if we dont get a block.
    while (millis() - startTime < cReloadTimeout) {
      if (gApds.readProximity() > cProximityThreshold) {  // high value means something is near
        break;
      }
    }
    if (!blockDetected) {
      DEBUG_PRINTLN("Timeout reached when waiting for block.");
      continue;  // skip over the block saving if we didn't see a block
    }

    // Read the color of the detected block and add it to the belt
    RGB blockColor = readGlobalColorSensor();
    addBlockToStackFromRGB(blocks, blockColor);
    DEBUG_PRINT("COLLECTED: ");
    Serial2.println(blockColorToString(predictColor(blockColor)));  // doesnt work with debugprintln...
  }
  // Turn beltmotor off
  gL2Motors.setM1Speed(0);
}



// Rotates the robot until it is aligned (facing squarely) with a wall.
void squareUpUsingProx(int speed) {
  float distanceLeft = pollRangefinder(cDistPin1);
  float distanceRight = pollRangefinder(cDistPin1);
  float tolerance = 0.2;

  // Loop to adjust orientation until the robot is squared with the wall
  while (abs(distanceRight - distanceLeft) > tolerance) {
    if (distanceLeft < distanceRight) {
      gMecanumMotors.setSpeeds(-speed, -speed, -speed, -speed);
    } else {
      gMecanumMotors.setSpeeds(speed, speed, speed, speed);
    }

    // Delay briefly to allow the rotation to take effect before remeasuring
    delay(100);

    // Update distances after adjustment
    distanceLeft = pollRangefinder(cDistPin1);
    distanceRight = pollRangefinder(cDistPin1);
  }

  // Stop all wheels once squared up with the wall
  gMecanumMotors.setSpeeds(0, 0, 0, 0);
  
}



// Centers the robot over the IR array.
void centerOnIrArray(int speed) {
  uint16_t position = gQtr.readLineBlack(gLineSensorValues);
  const uint16_t targetPosition = 3500;  // Assuming center is 3500 for 8 sensors
  const uint16_t tolerance = 50;         // Adjust this value based on how precise the centering needs to be

  // Loop to adjust position until the robot is centered on the line
  while (abs((int)position - (int)targetPosition) > tolerance) {
    if (position < targetPosition) {
      // If the sensor position is to the left of center, strafe right
      gMecanumMotors.setSpeeds(-speed, -speed, speed, speed);  // Strafe right
    } else {
      // If the sensor position is to the right of center, strafe left
      gMecanumMotors.setSpeeds(speed, speed, -speed, -speed);  // Strafe left
    }

    // Delay briefly to allow the movement to take effect before remeasuring
    delay(50);

    // Update position after adjustment
    position = gQtr.readLineBlack(gLineSensorValues);
  }

  // Stop all wheels once centered
  gMecanumMotors.setSpeeds(0, 0, 0, 0);
}



// Makes the robot physically push the button. It first drives forward to the correct distance for begging to push the button.
void pushButton(int speed) {
  const float targetProximityForward = 4;    // target proximity in meters or consistent unit for moving forward
  const float targetProximityBackward = 10;  // target proximity in meters or consistent unit for reversing

  // Drive forward until the proximity sensor reads less than 3 cm
  while (true) {
    float distanceLeft = pollRangefinder(cDistPin1);
    float distanceRight = pollRangefinder(cDistPin2);
    float avgDistance = (distanceLeft + distanceRight) / 2.0;

    if (avgDistance < targetProximityForward) {
      break;  // stop moving forward if within target proximity
    }

    gMecanumMotors.setSpeeds(speed, -speed, speed, -speed);  // Move forward
    delay(50);                                               // Wait for a bit before checking again
  }

  // Stop the robot
  gMecanumMotors.setSpeeds(0, 0, 0, 0);
  delay(250);  // Short delay before moving backward

  // Reverse until the proximity sensor reads less than 7 cm
  while (true) {
    float distanceLeft = pollRangefinder(cDistPin1);
    float distanceRight = pollRangefinder(cDistPin2);
    float avgDistance = (distanceLeft + distanceRight) / 2.0;

    if (avgDistance > targetProximityBackward) {
      break;  // stop moving backward if within target proximity
    }

    gMecanumMotors.setSpeeds(-speed, speed, -speed, speed);  // Move backward
    delay(50);                                               // Wait for a bit before checking again
  }

  // Finally, stop the robot
  gMecanumMotors.setSpeeds(0, 0, 0, 0);
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

// Drives for an amount of time after ramping up to a goal speed.
void runWheelMotorsWithBlockingDelay(int delayTime, float* targetWheelSpeeds) {
  if (!targetWheelSpeeds) {
    DEBUG_PRINTLN("Error: targetWheelSpeeds is null.");
  }

  DEBUG_PRINTLN("Running wheel motors with blocking delay and speed ramp.");

  // Map target wheel speeds from their current values to a scale suitable for the motor drivers before ramping.
  mapWheelSpeeds(targetWheelSpeeds, gDriveSpeed);  // map to global drive speed

  // Ramp speeds up to mapped target values over a period (e.g., 200 milliseconds)
  rampMotorSpeed(targetWheelSpeeds, 200, true);  // true ramps up

  // Wait for the specified delay time after ramping to the target speed.
  delay(delayTime);

  // Optionally, smoothly ramp down to 0 for a soft stop.
  rampMotorSpeed(targetWheelSpeeds, 200, false);  // false ramps down

  DEBUG_PRINTLN("Motors stopped.");
}



// Maps goal speeds to the motor libraries range (0 to 400) and sets the motors to those speeds. 
void runWheelMotorsDirectly(float* targetWheelSpeeds) {
  if (!targetWheelSpeeds) {
    DEBUG_PRINTLN("Error: targetWheelSpeeds is null.");
  }

  // Map target wheel speeds from their current values to a scale suitable for the motor drivers before ramping.
  float mappedSpeeds[cNumberOfWheels];
  mapWheelSpeeds(targetWheelSpeeds, gDriveSpeed);  // map to global drive speed
  gMecanumMotors.setSpeeds(targetWheelSpeeds[0], -targetWheelSpeeds[1], targetWheelSpeeds[2], -targetWheelSpeeds[3]);
}



// Ramps motor speed from 0 to targetSpeed over a specified duration
void rampMotorSpeed(float* targetWheelSpeeds, int rampDuration, bool rampDirection) {
  unsigned long rampStartTime = millis();
  unsigned long currentTime;
  float currentSpeed[cNumberOfWheels] = { 0, 0, 0, 0 };  // Start speeds at 0

  while (true) {
    currentTime = millis() - rampStartTime;
    float rampProgress = (float)currentTime / (float)rampDuration;

    if (rampProgress >= 1.0) {
      // If ramp is complete, ensure target speed is set
      if (rampDirection == true) {
        gMecanumMotors.setSpeeds(targetWheelSpeeds[0], -targetWheelSpeeds[1], targetWheelSpeeds[2], -targetWheelSpeeds[3]);
      } else {
        gMecanumMotors.setSpeeds(0, 0, 0, 0);
      }
      return;  // all done
    }

    // Calculate and set intermediate speeds
    for (int i = 0; i < cNumberOfWheels; i++) {
      if (rampDirection == true) {
        //ramp up
        currentSpeed[i] = targetWheelSpeeds[i] * rampProgress;
      } else {
        // ramp down
        currentSpeed[i] = targetWheelSpeeds[i] * (1 - rampProgress);
      }
    }
    gMecanumMotors.setSpeeds(currentSpeed[0], -currentSpeed[1], currentSpeed[2], -currentSpeed[3]);
    delay(10);  // Small delay to avoid updating too frequently
  }
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
