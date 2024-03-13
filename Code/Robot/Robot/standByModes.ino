/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
  * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */ 

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

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
  * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */ 

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

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
  * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */ 

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

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
  * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */ 

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