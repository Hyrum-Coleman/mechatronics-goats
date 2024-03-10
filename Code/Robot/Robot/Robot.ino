// Enable or disable debug prints
#define DEBUG_PRINTS_ENABLED true

#if DEBUG_PRINTS_ENABLED
#define DEBUG_PRINT(x) (Serial2.print(x))
#define DEBUG_PRINTLN(x) (Serial2.println(x))
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

// Include dependencies
#include <Arduino.h>
#include <ArduinoSTL.h>
#include <queue>
#include <stack>
#include <ArduinoJson.h>
#include <StreamUtils.h>
#include <DualTB9051FTGMotorShieldMod3230.h>
#include <L298NMotorDriverMega.h>
#include <QTRSensors.h>
#include <IRremote.hpp>
#include <Adafruit_APDS9960.h>
#include "Wheelbase.h"
#include "types.h"


// Global variables :(
// QUANTITIES
const int cNumberOfWheels = 4;
const uint8_t cSensorCount = 8;
const int cMaxBlocks = 5;
// PARAMETERS
const int cFilterWindowSize = 20;
int gDriveSpeed = 200;
int gRemoteControlDuration = 1000;
unsigned long gLastRCCommandTime = 0;
const unsigned long cRCCommandTimeout = 110;
const unsigned long cReloadTimeout = 5000;
const unsigned int cProximityThreshold = 50;
const float cHallReloadingThreshold = 540; // This needs to be tested by hand.
// PINS
const int cDistPin1 = A4;  // Left IR rangefinder sensor
const int cDistPin2 = A5;  // Right IR rangefinder sensor
const int cTopLimitSwitchPin = 53;
const int cBottomLimitSwitchPin = 52;
const int cIrRecievePin = 11;
const int cHallSensorPin = A3;

// Sensor globals
uint16_t gLineSensorValues[cSensorCount];
std::queue<float> gDistSensor1Readings;
std::queue<float> gDistSensor2Readings;
QTRSensors gQtr;
Adafruit_APDS9960 gApds;

// Motor globals
DualTB9051FTGMotorShieldMod3230 gMecanumMotors;
L298NMotorDriverMega gL2Motors(5, 34, 32, 6, 33, 35);
Wheelbase* gWheelbase = new Wheelbase(5.0625, 4.386, 2.559);

// For keeping track of previous standby state so we can return to it
States gLastStandbyState;

int main() {
  init();  // Initialize board itself
  Serial.begin(9600);
  Serial2.begin(9600);
  Serial2.setTimeout(10000);

  JsonDocument doc;

  // initialize both DualTB drivers
  gMecanumMotors.init();
  gMecanumMotors.enableDrivers();

  // initialize L298N
  gL2Motors.init();

  // initialize IR array
  gQtr.setTypeRC();
  gQtr.setSensorPins((const uint8_t[]){
                       36, 38, 40, 42, 43, 41, 39, 37 },
                     cSensorCount);

  // Start the IR Reciever
  IrReceiver.begin(cIrRecievePin, true);  // true for enable IR feedback

  if (!gApds.begin()) {
    DEBUG_PRINTLN("Initialization Failed :(");
  } else {
    DEBUG_PRINTLN("Device initialized!");
    //enable color sensing mode
    gApds.enableColor(true);
    gApds.enableProximity(true);
    DEBUG_PRINT("DEFAULT ADC GAIN: ");
    DEBUG_PRINTLN(gApds.getADCGain());
    DEBUG_PRINT("DEFAULT ADC INTEGRTION TIME: ");
    DEBUG_PRINTLN(gApds.getADCIntegrationTime());
    //gApds.setADCGain(APDS9960_AGAIN_64X); // trying this to fix sensitivity issue 
    //gApds.setADCIntegrationTime(200); 
  }
  setPinModes();

  loop(doc);
}

void loop(JsonDocument& doc) {
  // Control flow globals :(
  std::queue<Move>* moveQueue = new std::queue<Move>();
  std::queue<MicroMoves>* microMoveQueue = new std::queue<MicroMoves>();
  std::stack<Block>* blocks = new std::stack<Block>();
  States state = eStandbyIR;
  AdjustmentSubModes currentAdjustmentSubMode = eNotAdjusting;

  // LOOP BEGINS
  // -------------------------------------------------
  while (true) {
    switch (state) {
      case eStandbyJSON:
        standbyJSON(doc, moveQueue, state);
        gLastStandbyState = eStandbyJSON;
        break;
      case eStandbyIR:
        standbyIR(doc, moveQueue, blocks, state, currentAdjustmentSubMode);
        gLastStandbyState = eStandbyIR;
        break;
      case eMoving:
        executeMoveSequence(moveQueue);
        state = gLastStandbyState;  // return to the state we came from when done moving
        break;
      case eReloading:
        executeReload(blocks, microMoveQueue);
        state = gLastStandbyState;  // return to the state we came from when done reloading
        break;
      case eAdjustmentMode:
        executeAdjustmentMode(state, currentAdjustmentSubMode);
        break;
      case eSensorDumpMode:
        executeSensorDumpMode(state);
        break;
      case eStandbyRC:
        standbyRC(state);
        gLastStandbyState = eStandbyRC;
        break;
        // Other cases as needed
    }
  }
  // -------------------------------------------------
}

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

void standbyIR(JsonDocument& doc, std::queue<Move>* moveQueue, std::stack<Block>* blocks, States& state, AdjustmentSubModes& currentAdjustmentSubMode) {
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
      DEBUG_PRINTLN("PRINTING SENSOR DATA");
      debugPrintSensors();
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
    case RemoteButtons::eZero:         // move belt forwards
    case RemoteButtons::eOne:          // move belt forwards
      // For each of these cases, setup the move according to the button press
      move = setupMoveFromIRCommand((RemoteButtons)IrReceiver.decodedIRData.command);
      moveQueue->push(move);
      executeMoveSequence(moveQueue);
      break;
    case RemoteButtons::eFuncStop:  // Enter adjustment mode
      state = eAdjustmentMode;
      currentAdjustmentSubMode = eNotAdjusting;  // Reset to not adjusting
      DEBUG_PRINTLN("Entering adjustment mode");
      break;
    case RemoteButtons::eSeven:  // Color Sensor add to block queue
      DEBUG_PRINTLN("ADDING BLOCK TO QUEUE");
      RGB colorReading = readGlobalColorSensor();
      addToStackFromRGB(blocks, colorReading);
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

void executeAdjustmentMode(States& state, AdjustmentSubModes& currentAdjustmentSubMode) {
  if (!IrReceiver.decode()) {
    return;
  }

  switch ((RemoteButtons)IrReceiver.decodedIRData.command) {
    case RemoteButtons::eZero:
      currentAdjustmentSubMode = eAdjustingDriveSpeed;
      DEBUG_PRINTLN("Selected gDriveSpeed for adjustment.");
      break;
    case RemoteButtons::eOne:
      currentAdjustmentSubMode = eAdjustingRemoteControlDuration;
      DEBUG_PRINTLN("Selected gRemoteControlDuration for adjustment.");
      break;
    case RemoteButtons::eVolPlus:
      handleAdjustmentMode(currentAdjustmentSubMode, 400, 100, 10000, 500, [](int a, int b) {
        return std::min(a, b);
      });
      break;
    case RemoteButtons::eVolMinus:
      handleAdjustmentMode(currentAdjustmentSubMode, 0, -100, 0, -500, [](int a, int b) {
        return std::max(a, b);
      });
      break;
    case RemoteButtons::eFuncStop:
      state = eStandbyIR;                        // Go back to standby IR mode
      currentAdjustmentSubMode = eNotAdjusting;  // Reset adjustment mode
      DEBUG_PRINTLN("Exiting adjustment mode.");
      break;
  }
  IrReceiver.resume();
  delay(100);  //debounce
}

// these inputs need to be named better, but honestly I have no earthly idea what they're supposed to represent.
void handleAdjustmentMode(AdjustmentSubModes& currentAdjustmentSubMode, int input1, int input2, int input3, int input4, int (*pred)(int, int)) {
  switch (currentAdjustmentSubMode) {
    case eAdjustingDriveSpeed:
      gDriveSpeed = pred(input1, gDriveSpeed + input2);
      DEBUG_PRINT("gDriveSpeed changed to: ");
      DEBUG_PRINTLN(gDriveSpeed);
      break;
    case eAdjustingRemoteControlDuration:
      gRemoteControlDuration = pred(input3, gRemoteControlDuration + input4);
      DEBUG_PRINT("gRemoteControlDuration changed to: ");
      DEBUG_PRINTLN(gRemoteControlDuration);
      break;
  }
}

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
        calibrate(nextMove);
        break;
      default:
        DEBUG_PRINT("Unexpected moveType: ");
        DEBUG_PRINTLN(nextMove.moveType);
        break;
    }
  }
}

Move getNextMoveFromQueue(std::queue<Move>* queueToPopFrom) {
  Move retMove = queueToPopFrom->front();
  queueToPopFrom->pop();

  return retMove;
}

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

void runWheelMotorsDirectly(float* targetWheelSpeeds) {
  if (!targetWheelSpeeds) {
    DEBUG_PRINTLN("Error: targetWheelSpeeds is null.");
  }

  // Map target wheel speeds from their current values to a scale suitable for the motor drivers before ramping.
  float mappedSpeeds[cNumberOfWheels];
  mapWheelSpeeds(targetWheelSpeeds, gDriveSpeed);  // map to global drive speed
  gMecanumMotors.setSpeeds(targetWheelSpeeds[0], -targetWheelSpeeds[1], targetWheelSpeeds[2], -targetWheelSpeeds[3]);
}

void calibrate(Move nextMove) {
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

void mapWheelSpeeds(float* wheelSpeeds, unsigned long maxSpeed) {
  for (int i = 0; i < cNumberOfWheels; i++) {
    wheelSpeeds[i] = map(wheelSpeeds[i], -3.91, 3.91, -1 * maxSpeed, maxSpeed);
  }
}

// Function to ramp motor speed from 0 to targetSpeed over a specified duration
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

float pollRangefinder(int pin) {
  int sensorValue = analogRead(pin);
  float voltage = sensorValue * (5.0 / 1023.0);
  float distance = 33.9 - 69.5 * voltage + 62.3 * pow(voltage, 2) - 25.4 * pow(voltage, 3) + 3.83 * pow(voltage, 4);
  return distance;
}

float pollRangefinderWithSMA(int pin, std::queue<float>& readingsQueue) {
  int sensorValue = analogRead(pin);
  float voltage = sensorValue * (5.0 / 1023.0);
  float distance = 33.9 - 69.5 * voltage + 62.3 * pow(voltage, 2) - 25.4 * pow(voltage, 3) + 3.83 * pow(voltage, 4);

  // Add new reading to the queue
  if (readingsQueue.size() >= cFilterWindowSize) {
    readingsQueue.pop();  // Remove the oldest reading if we've reached capacity
  }
  readingsQueue.push(distance);

  // Calculate the moving average
  float sum = 0;
  for (std::queue<float> tempQueue = readingsQueue; !tempQueue.empty(); tempQueue.pop()) {
    sum += tempQueue.front();
  }
  float averageDistance = sum / readingsQueue.size();

  return averageDistance;
}

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

void addToStackFromRGB(std::stack<Block>* blocks, RGB rgb) {
  Block newBlock = createBlock(rgb);

  addBlockToBelt(blocks, newBlock);
}

void executeReload(std::stack<Block>* blocks, std::queue<MicroMoves>* microMovesQueue) {
  // Drive belt backwards to collect blocks as they enter the belt.
  // In future, make it only drive when we need it to. I just dont know the timings yet.
  gL2Motors.setM1Speed(-400);
  // While our belt is not full of blocks,
  while (blocks->size() < cMaxBlocks) {

    // Uncomment this when rest of reloading works
    // If the other team has pushed the button, we should wait until its ready to be pushed (using hall effect sensor)
    if (getCurrentHallVoltage() < cHallReloadingThreshold) {  // check < vs > here
      // if the magnet is not detected, the platform is up too high, meaning it is not yet ready for reloading.
      // in that instance, we skip this iteration of the loop and wait until it is detected.
      continue;
    }

    // Get in button pushing position
    // --> square up using proximity sensors
    microMovesQueue->push(eSquareUpUsingProx);
    // --> drive sideways slowly until in center of IR array
    microMovesQueue->push(eCenterOnIrArray);
    // Push button by driving forwards and then backwards (poll distance sensor?)
    microMovesQueue->push(ePushButton);
    // Dewit
    executeMicroMoves(microMovesQueue);

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
    addToStackFromRGB(blocks, blockColor);
  }
  // Turn beltmotor off
  gL2Motors.setM1Speed(0);
}


void executeMicroMoves(std::queue<MicroMoves>* microMovesQueue) {
  while (!microMovesQueue->empty()) {
    MicroMoves move = microMovesQueue->front();
    microMovesQueue->pop();
    switch (move) {
      case eSquareUpUsingProx:
        squareUpUsingProx();
        break;
      case eCenterOnIrArray:
        centerOnIrArray();
        break;
      case ePushButton:
        pushButton();
        break;
      // Add cases for other micro-moves as needed
      default:
        DEBUG_PRINTLN("Invalid micro move.");
        break;
    }
  }
}

void squareUpUsingProx() {
}

void centerOnIrArray() {
}

void pushButton() {
}

float getCurrentHallVoltage() {
  float hallVoltage = analogRead(cHallSensorPin);
  return hallVoltage;
}

void setPinModes() {
  pinMode(cTopLimitSwitchPin, INPUT);
  pinMode(cBottomLimitSwitchPin, INPUT);
  pinMode(cHallSensorPin, INPUT);
  pinMode(cDistPin1, INPUT);
  pinMode(cDistPin2, INPUT);
}

void executeSensorDumpMode(States& state) {
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

// todo: make this not shit.
// for now, it works 100% of the time, so thats good :)
BlockColor predictColor(RGB colorReading) {
  int total = colorReading.r + colorReading.g + colorReading.b;

  float r_norm = (float)colorReading.r / total;
  float g_norm = (float)colorReading.g / total;
  float b_norm = (float)colorReading.b / total;

  if (r_norm > 0.6 && g_norm < 0.3 && b_norm < 0.3) {
    return BlockColor::Red;
  } else if (r_norm > 0.4 && g_norm > 0.28 && b_norm < 0.2) {
    return BlockColor::Yellow;
  } else {  // hack to get around not detecting blue very well
    if (total < 10) {
      return BlockColor::Blue;
    }
    if (total > 30) {
      if (b_norm > 0.4 && g_norm < 0.35 && r_norm < 0.35) {
        return BlockColor::Blue;
      }
    }
  }
  return BlockColor::None;
}

const char* blockColorToString(BlockColor color) {
  switch (color) {
    case BlockColor::Red: return "Red";
    case BlockColor::Yellow: return "Yellow";
    case BlockColor::Blue: return "Blue";
    case BlockColor::None: return "None";
    default: return "Unknown";
  }
}

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
  }
  else {
    Serial2.print(" | Magnet: No");
  }

  Serial2.println("");

  delay(200);
}
