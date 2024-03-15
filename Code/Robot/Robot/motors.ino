// For every move in a queue, do the move.
void executeMoveSequence(std::queue<Move>* moveQueue) {
  while (!moveQueue->empty()) {
    Move nextMove = getNextMoveFromQueue(moveQueue);
    switch (nextMove.moveType) {
      case eFreeDrive:
        executeFreeDrive(nextMove);
        break;
      case eLineFollow:
        executeLineFollow(nextMove);
        break;
      case eScissor:
        executeScissor(nextMove);
        break;
      case eBelt:
        executeBelt(nextMove);
        break;
      case eCalibrate:
        calibrateIrArray(nextMove);
        break;
      default:
        DEBUG_PRINT("Unexpected moveType: ");
        DEBUG_PRINTLN(nextMove.moveType);
        break;
    }
  }
}

void labStylePositionControl(Pose goalPose, float driveTime) {
  Velocities fwdVelocities;
  Pose nextPose;
  float thresh = 1.0;
  float thresh_th = 0.2;
  float Kp = 1.0;
  // Array to store calculated errors
  float wheelSpeedsDes[cNumberOfWheels];
  // Array to store measured wheel speeds
  float odomWheelSpeeds[cNumberOfWheels];
  // Array to store control signals for the wheels
  float controlSignals[cNumberOfWheels];

  // These are analagous to omega_des in the lab9 code
  float vx_des = (goalPose.x - gRobotPose.x) / driveTime;
  float vy_des = (goalPose.y - gRobotPose.y) / driveTime;
  float vth_des = (goalPose.theta - gRobotPose.theta) / driveTime; 

  double tOld = micros() / 1000000.;
  double t, deltaT;

  float err_x, err_y, err_th;
  float u_x, u_y, u_th;

  do {
    t = micros() / 1000000.;
    deltaT = t - tOld;

    nextPose.x += vx_des * deltaT;
    nextPose.y += vy_des * deltaT;
    nextPose.theta += vth_des * deltaT;

    // generate errors between present pose and next pose
    err_x = nextPose.x - gRobotPose.x;
    err_y = nextPose.y - gRobotPose.y;
    err_th = nextPose.theta - gRobotPose.theta;

    // PID will go here. These are the 'control signals' before transforming to robot coordinates and mapping to correct units.
    u_x = Kp * err_x;
    u_y = Kp * err_y;
    u_th = Kp * err_th;

    // Transform control signals to robot coordinates
    gWheelbase->computeWheelSpeeds(u_x, u_y, u_th, controlSignals);
    // Account for the polarity of our motors.
    controlSignals[1] *= -1; //( [] [-] [] [-] )
    controlSignals[3] *= -1;

    // Convert control signals from rad/sec to motor driver units
    radSecToMotorDriverSpeeds(controlSignals);

    // Set the speeds to the control signals
    gMecanumMotors.setSpeeds(controlSignals[0], controlSignals[1], controlSignals[2], controlSignals[3]);

    // Get current wheel speeds from encoders
    odomWheelSpeeds[0] = gWheel1Manager.getWheelSpeedRadPerSec(); // flip is taken care of in the wheel manager
    odomWheelSpeeds[1] = gWheel2Manager.getWheelSpeedRadPerSec();
    odomWheelSpeeds[2] = gWheel3Manager.getWheelSpeedRadPerSec();
    odomWheelSpeeds[3] = gWheel4Manager.getWheelSpeedRadPerSec();

    // Calculate current velocity based on wheel speeds and fwd kinematics
    // modifies fwdVelocities in place
    gWheelbase->computeVelocities(odomWheelSpeeds, fwdVelocities.xDot, fwdVelocities.yDot, fwdVelocities.thetaDot);

    // Update pose based on updated velocities
    gRobotPose.update_pos(fwdVelocities.xDot, fwdVelocities.yDot, fwdVelocities.thetaDot);

    tOld = t;
  } while (abs(err_x) > thresh || abs(err_y) > thresh || abs(err_th) > thresh_th); // need different thresh for theta beacuse rads
}

// Experimental function that controls our robot with desired velocities until a condition is met.
void executeVelocitiesUntilCondition(const Velocities& v, DrivingTerminationCondition term) {
  // Array to store wheel speeds calculated based on desired velocities
  float wheelSpeeds[cNumberOfWheels];
  // Array to store odometry speeds (for use in while loop)
  float odomWheelSpeeds[cNumberOfWheels];
  // Array for our command signals. This is the result of our PID algorithm.
  float controlSignals[cNumberOfWheels];
  // To store our velocities that the inverse kinematics mutates
  Velocities ikVelocities;

  // Calculate required wheel speeds to achieve desired velocities
  gWheelbase->computeWheelSpeeds(v.xDot , v.yDot, v.thetaDot, wheelSpeeds);
  // now take the integral of this to get position?
  
  // Wheelspeeds now contains goal rad/sec values for each wheel.
  // Account for the polarity of our motors.
  wheelSpeeds[1] *= -1;
  wheelSpeeds[3] *= -1;
  // We now want to command the motors to those speeds. 
  // More specifically, we want to ramp smoothly up to the goal speeds while numerically integrating our position until we reach the termination condition.
  // Initialize the ramp with 1 second ramp up time.
  // It will ramp up our speeds based on the current time and the ramp time. 
  // If our current time exceeds the ramp time, the original goal will be returned.
  MotorRamp ramp(wheelSpeeds, 1000);
  while (!term.tripped) { // while the condition has not been met
    // Update odometry
    // Todo: make wheelmanager manage all 4 wheels (take in an int to choose which wheel)
    odomWheelSpeeds[0] = gWheel1Manager.getWheelSpeedRadPerSec(); // flip is taken care of in the wheel manager
    odomWheelSpeeds[1] = gWheel2Manager.getWheelSpeedRadPerSec();
    odomWheelSpeeds[2] = gWheel3Manager.getWheelSpeedRadPerSec();
    odomWheelSpeeds[3] = gWheel4Manager.getWheelSpeedRadPerSec();

    // Uses measured wheel speeds to calculate robot velocity with forward kinematics. Mutates ikVelocities in place.
    gWheelbase->computeVelocities(odomWheelSpeeds, ikVelocities.xDot, ikVelocities.yDot, ikVelocities.thetaDot);

    // Automatically updates predicted position using the ik velocities
    gRobotPose.update_pos(ikVelocities.xDot, ikVelocities.yDot, ikVelocities.thetaDot);

    // Map controlSignals to our ramp function. Internally based on time and the goal speeds set when instantiating MotorRamp.
    ramp.setControlSignals(controlSignals); // mutates controlsignals

    // Before calling setSpeeds, we map to -400, 400
    radSecToMotorDriverSpeeds(controlSignals);
    // Now we finally set the speeds after calculating them in rad/s with IK and ramping them.
    gMecanumMotors.setSpeeds(controlSignals[0], controlSignals[1], controlSignals[2], controlSignals[3]);

    // Call function that checks the condition based on the condition type. It takes in term.
    // Inside that function, the termination condition gets tripped if the condition is met. This breaks the loop and we are done.
    switch (term.type) {
      case TerminationType::LineCovered:
        // function that takes in term and switches it if the condition is met
        break;
      case TerminationType::LineCentered:
        break;
      case TerminationType::AverageRangeFinderDistance:
        break;
      case TerminationType::DistanceTraveled:
        break;
      case TerminationType::AngleReached:
        break;
      case TerminationType::TimeExpired:
        break;
      default:
        break;
    }
  }
}


// Takes a freedrive move and makes it happen
void executeFreeDrive(Move nextMove) {
  float wheelSpeeds[cNumberOfWheels];  // Initialize motor speeds
  int delayTime = nextMove.params.freedriveParams.duration;
  int y_velocity = 0;
  int x_velocity = 0;
  int omega = 0;

  switch (nextMove.params.freedriveParams.direction) {
    case eForwards:
      y_velocity = 10;
      break;
    case eLeft:
      x_velocity = -10;
      break;
    case eBackwards:
      y_velocity = -10;
      break;
    case eRight:
      x_velocity = 10;
      break;
    case eCCW:
      omega = 1.059;
      break;
    case eCW:
      omega = -1.059;
      break;
    default:
      DEBUG_PRINTLN("Unexpected input in direction switch for freedrive.");
      break;
  }

  gWheelbase->computeWheelSpeeds(x_velocity, y_velocity, omega, wheelSpeeds);
  runWheelMotorsWithBlockingDelay(delayTime, wheelSpeeds);
}



// Takes a linefollowing move and makes it happen
void executeLineFollow(Move nextMove) {
  float targetDistance = nextMove.params.linefollowParams.stopDistance;
  int baseSpeed = nextMove.params.linefollowParams.speed;
  int lastError = 0;                               // Variable to store the last error for the derivative term
  double Kp = (1.0 / 20.0) * (baseSpeed / 200.0);  // Proportional gain
  double Kd = 0.01;

  unsigned long lastMotorUpdateTime = 0;          // Stores the last time the motors were updated
  const unsigned long motorUpdateInterval = 100;  // Update motors every 100 milliseconds

  while (true) {
    // Poll the rangefinders continuously
    float distanceLeft = getDistFromRangeFinderFiltered(cDistPin1, DistanceCalibrationMaterial::Cardboard);
    float distanceRight = getDistFromRangeFinderFiltered(cDistPin2, DistanceCalibrationMaterial::Cardboard);

    // If close enough to the wall, stop
    if (distanceLeft <= targetDistance || distanceRight <= targetDistance) {
      gMecanumMotors.setSpeeds(0, 0, 0, 0);  // Stop the robot
      break;                                 // Exit the loop
    }

    unsigned long currentMillis = millis();

    // Non-blocking delay logic for motor speed adjustments
    if (currentMillis - lastMotorUpdateTime >= motorUpdateInterval) {
      // Perform line following logic
      uint16_t position = gQtr.readLineBlack(gLineSensorValues);
      int error = position - 3500;         // Center is 3500 for 8 sensors
      int derivative = error - lastError;  // Calculate derivative. This is over 100ms because thats the motor update interval.

      int leftSpeed = baseSpeed + (Kp * error) + (Kd * derivative);
      int rightSpeed = baseSpeed - (Kp * error) - (Kd * derivative);

      // Set motor speeds based on line position
      gMecanumMotors.setSpeeds(leftSpeed, -rightSpeed, leftSpeed, -rightSpeed);

      lastError = error;                    // Update lastError for the next iteration
      lastMotorUpdateTime = currentMillis;  // Update the time of last motor update
    }

    // The loop now continues without delay, allowing for continuous sensor polling
  }
}



// NOTE: THE DIRECTION OF THE MOTOR TO GO UP VS DOWN MAY NEED TO BE CHANGED!!!
// If switches dont get triggered, this times out to avoid getting stuck in a loop
// Executes a scissor lift move
void executeScissor(Move nextMove) {
  unsigned long targetHeight = nextMove.params.scissorParams.direction;
  unsigned long startTime = millis();  // Capture the start time
  unsigned long timeout = 3000;        // Set timeout

  if (targetHeight == 1) {
    DEBUG_PRINTLN("MOVING PLATFORM UP");
    // Move towards the top limit switch
    gL2Motors.setM2Speed(320);
    while (digitalRead(cTopLimitSwitchPin) == HIGH) {
      // Check if timeout is exceeded
      if (millis() - startTime > timeout) {
        DEBUG_PRINTLN("Timeout reached while moving up");
        break;  // Exit the loop if the timeout is exceeded
      }
      delay(10);  // Small delay to prevent too rapid polling
    }
  } else if (targetHeight == 0) {
    DEBUG_PRINTLN("MOVING PLATFORM DOWN");
    // Move towards the bottom limit switch
    gL2Motors.setM2Speed(-320);
    while (digitalRead(cBottomLimitSwitchPin) == HIGH) {
      // Check if timeout is exceeded
      if (millis() - startTime > timeout) {
        DEBUG_PRINTLN("Timeout reached while moving down");
        break;  // Exit the loop if the timeout is exceeded
      }
      delay(10);  // Small delay to prevent too rapid polling
    }
  }

  gL2Motors.setM2Speed(0);  // Stop the motor once the limit switch is reached or timeout occurs
}



// Executes a belt move
void executeBelt(Move nextMove) {
  unsigned long duration = nextMove.params.beltParams.duration;  // Duration in milliseconds
  bool direction = nextMove.params.beltParams.direction;         // Direction (1 is forward, 0 is backward)

  // Determine speed based on direction
  int speed = direction ? 400 : -400;  // Assume positive speed for forward, negative for backward

  if (speed == 400) {
    DEBUG_PRINTLN("MOVING BELT FORWARD");
  } else {
    DEBUG_PRINTLN("MOVING BELT BACKWARD");
  }

  gL2Motors.setM1Speed(speed);  // Set speed and direction
  delay(duration);              // Run for specified duration
  gL2Motors.setM1Speed(0);      // Stop the belt

  DEBUG_PRINTLN("Done moving belt.");
}

// Gets the next move from the queue of moves, no flipping way
Move getNextMoveFromQueue(std::queue<Move>* queueToPopFrom) {
  Move retMove = queueToPopFrom->front();
  queueToPopFrom->pop();

  return retMove;
}

// Generalizes handling remote buttons for moves
Move setupMoveFromIRCommand(RemoteButtons command) {
  Move move;
  switch (command) {
    case RemoteButtons::eVolPlus:
      move.moveType = MoveType::eFreeDrive;
      move.params.freedriveParams.direction = Directions::eForwards;
      move.params.freedriveParams.duration = gRemoteControlDuration;
      break;
    case RemoteButtons::eBack:
      move.moveType = MoveType::eFreeDrive;
      move.params.freedriveParams.direction = Directions::eLeft;
      move.params.freedriveParams.duration = gRemoteControlDuration;
      break;
    case RemoteButtons::eFastForward:
      move.moveType = MoveType::eFreeDrive;
      move.params.freedriveParams.direction = Directions::eRight;
      move.params.freedriveParams.duration = gRemoteControlDuration;
      break;
    case RemoteButtons::eDown:
      move.moveType = MoveType::eFreeDrive;
      move.params.freedriveParams.direction = Directions::eCCW;
      move.params.freedriveParams.duration = gRemoteControlDuration;
      break;
    case RemoteButtons::eVolMinus:
      move.moveType = MoveType::eFreeDrive;
      move.params.freedriveParams.direction = Directions::eBackwards;
      move.params.freedriveParams.duration = gRemoteControlDuration;
      break;
    case RemoteButtons::eUp:
      move.moveType = MoveType::eFreeDrive;
      move.params.freedriveParams.direction = Directions::eCW;
      move.params.freedriveParams.duration = gRemoteControlDuration;
      break;
    //Uncomment this when limit switches are working
    //case RemoteButtons::eTwo:  // move platform up
    //  move.moveType = MoveType::eScissor;
    //  move.params.scissorParams.direction = 1;
    //  break;
    //case RemoteButtons::eEight:  // move platform down
    //  move.moveType = MoveType::eScissor;
    //  move.params.scissorParams.direction = 0;
    //  break;
    case RemoteButtons::eFour:  // move belt backwards
      move.moveType = MoveType::eBelt;
      move.params.beltParams.direction = 1;
      move.params.beltParams.duration = gRemoteControlDuration;
      break;
    case RemoteButtons::eSix:  // move belt forwards
      move.moveType = MoveType::eBelt;
      move.params.beltParams.direction = 0;
      move.params.beltParams.duration = gRemoteControlDuration;
      break;
    case RemoteButtons::eZero:  // move belt forwards
      move.moveType = MoveType::eCalibrate;
      move.params.calibrationParams.duration = 3000;  // make this changable in the configuration mode
      break;
    case RemoteButtons::eOne:
      move.moveType = MoveType::eLineFollow;
      move.params.linefollowParams.speed = gDriveSpeed;
      move.params.linefollowParams.stopDistance = 10;  // make this changable in the configuration mode
    default:
      // Set to a default move or log an error
      break;
  }
  return move;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

// Does the whole reload sequence.
// At start: robot is near the reloader over the line.
// At end: robot is near the reloader over the line, but now has a full platform of blocks.
// TODO: need to add platform height logic. Cant yet because no limit switches yet.
void executeReload(std::stack<Block>* blocks) {
  // Drive belt backwards to collect blocks as they enter the belt.
  // In future, make the belt only drive when we need it to. I just dont know the timings yet.
  bool linedUp = false;
  gL2Motors.setM1Speed(400);
  // While our belt is not full of blocks,
  while (blocks->size() < cMaxBlocks) {
    if (!linedUp) {
      DEBUG_PRINTLN("SQUARING UP");
      //squareUpUsingProx(70);
      delay(500);

      DEBUG_PRINTLN("CENTERING");
      //centerOnIrArray(70);
      delay(500);

      DEBUG_PRINTLN("SQUARING UP AGAIN");
      //squareUpUsingProx(70);

      linedUp = true;
    }

    // Uncomment this when rest of reloading works
    // If the other team has pushed the button, we should wait until its ready to be pushed (using hall effect sensor)
    if (!isMagnetDetected()) {
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
  float distanceLeft = getDistFromRangeFinderFiltered(cDistPin1, DistanceCalibrationMaterial::Cardboard);
  float distanceRight = getDistFromRangeFinderFiltered(cDistPin2, DistanceCalibrationMaterial::Cardboard);
  float tolerance = 0.05;

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
    distanceLeft = getDistFromRangeFinderFiltered(cDistPin1, DistanceCalibrationMaterial::Cardboard);
    distanceRight = getDistFromRangeFinderFiltered(cDistPin2, DistanceCalibrationMaterial::Cardboard);
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
  const float targetProximityForward = 0.4; //inches
  const float targetProximityBackward = 1.75;

  // Drive forward until the proximity sensor reads less than 3 cm
  while (true) {
    float distanceLeft = getDistFromRangeFinderFiltered(cDistPin1, DistanceCalibrationMaterial::Cardboard);
    float distanceRight = getDistFromRangeFinderFiltered(cDistPin2, DistanceCalibrationMaterial::Cardboard);
    float avgDistance = (distanceLeft + distanceRight) / 2.0;

    if (avgDistance < targetProximityForward) {
      // THIS LOGIC NEEDS TO BE CHANGED. WE NEVER GET WITHIN RANGE IF WE BUMP INTO IT.
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
    float distanceLeft = getDistFromRangeFinderFiltered(cDistPin1, DistanceCalibrationMaterial::Cardboard);
    float distanceRight = getDistFromRangeFinderFiltered(cDistPin2, DistanceCalibrationMaterial::Cardboard);
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
