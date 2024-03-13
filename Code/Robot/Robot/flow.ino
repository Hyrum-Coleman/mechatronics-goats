void read_serial(JsonDocument& doc) {
  ReadLoggingStream loggingStream(Serial2, Serial);
  DeserializationError error = deserializeJson(doc, loggingStream);

  // Test if parsing succeeds.
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }
}



void parseJsonIntoQueue(std::queue<Move>* moveQueue, JsonDocument& doc) {
  for (JsonObject obj : doc["g"].as<JsonArray>()) {  // g for go
    Move currentMove;

    // Populate move structs params based on move type
    MoveType moveType = obj["type"].as<MoveType>();

    currentMove.moveType = moveType;
    switch (currentMove.moveType) {
      case MoveType::eFreeDrive:
        currentMove.params.freedriveParams.direction = obj["direction"].as<Directions>();
        currentMove.params.freedriveParams.duration = obj["duration"];
        break;
      case MoveType::eLineFollow:
        currentMove.params.linefollowParams.stopDistance = obj["stopDistance"];
        currentMove.params.linefollowParams.speed = obj["speed"];
        break;
      case MoveType::eScissor:
        currentMove.params.scissorParams.direction = obj["direction"];
        break;
      case MoveType::eBelt:
        currentMove.params.beltParams.direction = obj["direction"];
        currentMove.params.beltParams.duration = obj["duration"];
        break;
      case MoveType::eCalibrate:
        currentMove.params.calibrationParams.duration = obj["duration"];
        break;
      default:
        DEBUG_PRINTLN("Unexpected type");
    };
    moveQueue->push(currentMove);
  }
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

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
    float distanceLeft = pollRangefinderWithSMA(cDistPin1, gDistSensor1Readings);
    float distanceRight = pollRangefinderWithSMA(cDistPin2, gDistSensor2Readings);

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

void standbyJSON(JsonDocument& doc, std::queue<Move>* moveQueue, States& state) {
  DEBUG_PRINT("STANDBY JSON... <");
  DEBUG_PRINT(millis() / 1000.0);
  DEBUG_PRINTLN(">");

  // Check IR Receiver specifically for the power button press to toggle state
  if (IrReceiver.decode()) {
    if ((RemoteButtons)IrReceiver.decodedIRData.command == RemoteButtons::ePwr) {
      state = eStandbyIR;
      DEBUG_PRINTLN("Cycle state: Switching to IR mode");
    }
    IrReceiver.resume();
    delay(100);  //debounce
    return;      // Early return to avoid JSON processing if power button was pressed
  }

  // Continue with JSON processing only if the power button was not pressed
  read_serial(doc);                             // Attempt to read and parse JSON from Serial
  if (!doc.isNull() && doc.containsKey("g")) {  // Check if JSON contains expected data
    parseJsonIntoQueue(moveQueue, doc);         // Parse commands into move queue
    state = eMoving;                            // Switch to moving state to execute parsed commands
  }
}



void standbyIR(JsonDocument& doc, std::queue<Move>* moveQueue, std::stack<Block>* blocks, States& state) {
  DEBUG_PRINT("STANDBY IR... <");
  DEBUG_PRINT(millis() / 1000.0);
  DEBUG_PRINTLN(">");

  if (!IrReceiver.decode()) {
    return;
  }

  Move move;
  switch ((RemoteButtons)IrReceiver.decodedIRData.command) {
    DEBUG_PRINTLN(IrReceiver.decodedIRData.command);
    case RemoteButtons::ePwr:  // Toggle state between JSON and IR standby modes
      state = eStandbyRC;
      DEBUG_PRINTLN("Cycle state: Switching to RC mode");
      break;
    // this case needs to be here and I have no idea why.
    case RemoteButtons::eThree:  // poll all sensors for testing and data collection
      DEBUG_PRINTLN("CALIBRATING COLORS");
      calibrateColorSensor();
      break;
    case RemoteButtons::eForward:  // run the course code
      DEBUG_PRINTLN("BEGINNING RELOAD TEST");
      move.moveType = MoveType::eLineFollow;  // linefollow up to reloader. This is a placeholder sorta
      move.params.linefollowParams.speed = gDriveSpeed;
      move.params.linefollowParams.stopDistance = 10;
      moveQueue->push(move);
      executeMoveSequence(moveQueue);
      executeReload(blocks);
      break;
    case RemoteButtons::eVolPlus:      // Drive forwards
    case RemoteButtons::eBack:         // Drive left
    case RemoteButtons::eFastForward:  // Drive right
    case RemoteButtons::eDown:         // Rotate counterclockwise
    case RemoteButtons::eVolMinus:     // Drive backwards
    case RemoteButtons::eUp:           // Rotate clockwise
    case RemoteButtons::eTwo:          // move platform up
    case RemoteButtons::eEight:        // move platform down
    case RemoteButtons::eFour:         // move belt backwards
    case RemoteButtons::eSix:          // move belt forwards
    case RemoteButtons::eZero:         // Calibrate line follower
    case RemoteButtons::eOne:          // Line follow
      // For each of these cases, setup the move according to the button press
      move = setupMoveFromIRCommand((RemoteButtons)IrReceiver.decodedIRData.command);
      moveQueue->push(move);
      executeMoveSequence(moveQueue);
      break;
    case RemoteButtons::eFuncStop:  // Enter adjustment mode
      DEBUG_PRINTLN("Adjustment mode has been removed :( sorry");
      break;
    case RemoteButtons::eSeven:  // Color Sensor add to block queue
      DEBUG_PRINTLN("ADDING BLOCK TO QUEUE");
      RGB colorReading = readGlobalColorSensor();
      addBlockToStackFromRGB(blocks, colorReading);
      break;
    case RemoteButtons::eFive:  // Inspect stack one block at a time
      if (blocks->empty()) {
        DEBUG_PRINTLN("Stack is empty");
        break;
      }
      Block topBlock = getNextBlock(blocks);
      DEBUG_PRINT("Block at top of stack: ");
      DEBUG_PRINTLN(blockColorToString(topBlock.color));
      break;
    default:
      DEBUG_PRINTLN("IR Command not handled.");
      break;
  }
  IrReceiver.resume();
  delay(100);  //debounce
}



void standbyRC(States& state) {
  DEBUG_PRINT("STANDBY RC... <");
  DEBUG_PRINT(millis() / 1000.0);
  DEBUG_PRINTLN(">");

  if (!IrReceiver.decode()) {
    if (millis() - gLastRCCommandTime > cRCCommandTimeout) {
      gMecanumMotors.setSpeeds(0, 0, 0, 0);  // Set speeds to 0 after timeout
      gL2Motors.setSpeeds(0, 0);
      gLastRCCommandTime = millis();  // update last command time to avoid constantly setting wheel speeds to 0
    }
    return;
  }

  float wheelSpeeds[cNumberOfWheels];
  switch ((RemoteButtons)IrReceiver.decodedIRData.command) {
    case RemoteButtons::ePwr:  // Toggle state between JSON and IR standby modes
      state = eSensorDumpMode;
      DEBUG_PRINTLN("Cycle state: Switching to sensor dump mode");
      break;
    case RemoteButtons::eVolPlus:  // Drive forwards
      gWheelbase->computeWheelSpeeds(0, 10, 0, wheelSpeeds);
      runWheelMotorsDirectly(wheelSpeeds);
      break;

    case RemoteButtons::eBack:  // Drive left
      gWheelbase->computeWheelSpeeds(-10, 0, 0, wheelSpeeds);
      runWheelMotorsDirectly(wheelSpeeds);
      break;

    case RemoteButtons::eFastForward:  // Drive right
      gWheelbase->computeWheelSpeeds(10, 0, 0, wheelSpeeds);
      runWheelMotorsDirectly(wheelSpeeds);
      break;

    case RemoteButtons::eDown:  // Rotate counterclockwise
      gWheelbase->computeWheelSpeeds(0, 0, 1.059, wheelSpeeds);
      runWheelMotorsDirectly(wheelSpeeds);
      break;

    case RemoteButtons::eVolMinus:  // Drive backwards
      gWheelbase->computeWheelSpeeds(0, -10, 0, wheelSpeeds);
      runWheelMotorsDirectly(wheelSpeeds);
      break;

    case RemoteButtons::eUp:  // Rotate clockwise
      gWheelbase->computeWheelSpeeds(0, 0, -1.059, wheelSpeeds);
      runWheelMotorsDirectly(wheelSpeeds);
      break;
    case RemoteButtons::eSix:
      gL2Motors.setSpeeds(-400, 0);
      break;
    case RemoteButtons::eFour:
      gL2Motors.setSpeeds(400, 0);
      break;
    case RemoteButtons::eTwo:
      gL2Motors.setSpeeds(0, 320);
      break;
    case RemoteButtons::eEight:
      gL2Motors.setSpeeds(0, -320);
      break;
    // Add additional case handlers as needed
    default:
      DEBUG_PRINTLN("IR Command not handled.");
      break;
  }
  IrReceiver.resume();
  gLastRCCommandTime = millis();
}



// The mode where sensor values are printed continuously. 
void standbySensorDump(States& state) {
  // Check IR Receiver specifically for the power button press to toggle state
  if (IrReceiver.decode()) {
    if ((RemoteButtons)IrReceiver.decodedIRData.command == RemoteButtons::ePwr) {
      state = eStandbyJSON;
      DEBUG_PRINTLN("Cycle state: Switching to Json mode");
    }
    IrReceiver.resume();
    delay(100);  //debounce
    return;
  }
  debugPrintSensors();
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

