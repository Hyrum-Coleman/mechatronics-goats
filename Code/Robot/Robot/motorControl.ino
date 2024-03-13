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

