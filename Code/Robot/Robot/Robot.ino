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
#include <ArduinoJson.h>
#include <StreamUtils.h>
#include <DualTB9051FTGMotorShieldMod3230.h>
#include <L298NMotorDriverMega.h>
#include <QTRSensors.h>
#include <queue>
#include "Wheelbase.h"
#include "types.h"
// these are both for IR reciever library
//#include "PinDefinitionsAndMore.h"
#include <IRremote.hpp>  // include the library

// Global variables :(
// QUANTITIES
const int cNumberOfWheels = 4;
const uint8_t cSensorCount = 8;
// PARAMETERS
const int cFilterWindowSize = 20;
int gDriveSpeed = 200;
int gRemoteControlDuration = 1000;
unsigned long gLastRCCommandTime = 0;
const unsigned long cRCCommandTimeout = 110;
AdjustmentSubModes gCurrentAdjustmentSubMode = eNotAdjusting;
// PINS
const int cDistPin1 = A4;              // Left IR rangefinder sensor
const int cDistPin2 = A5;              // Right IR rangefinder sensor
const int cTopLimitSwitchPin = 53;     // Replace XX with the actual pin number
const int cBottomLimitSwitchPin = 52;  // Replace YY with the actual pin number
const int cIrRecievePin = 11;          // IR Reciever

// Sensor globals
uint16_t sensorValues[cSensorCount];
std::queue<float> gDistSensor1Readings;
std::queue<float> gDistSensor2Readings;
QTRSensors gQtr;

// Motor globals
DualTB9051FTGMotorShieldMod3230 gMecanumMotors;
L298NMotorDriverMega gL2Motors(5, 34, 32, 6, 33, 35);
Wheelbase* gWheelbase = new Wheelbase(5.0625, 4.386, 2.559);

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

  // ...
  setPinModes();

  loop(doc);
}

void loop(JsonDocument& doc) {
  // Control flow globals :(
  std::queue<Move>* moveQueue = new std::queue<Move>();
  States state = eStandbyIR;

  // LOOP BEGINS
  // -------------------------------------------------
  while (true) {
    switch (state) {
      case eStandbyJSON:
        standbyJSON(doc, moveQueue, state);
        break;
      case eStandbyIR:
        standbyIR(doc, moveQueue, state);
        break;
      case eMoving:
        executeMoveSequence(moveQueue);
        state = eStandbyIR;  // Return to the default IR standby mode after executing moves
        break;
      case eAdjustmentMode:
        executeAdjustmentMode(state);
        break;
      case eStandbyRC:
        standbyRC(state);
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

void standbyIR(JsonDocument& doc, std::queue<Move>* moveQueue, States& state) {
  DEBUG_PRINT("STANDBY IR... <");
  DEBUG_PRINT(millis() / 1000.0);
  DEBUG_PRINTLN(">");

  if (IrReceiver.decode()) {
    Move move;
    switch ((RemoteButtons)IrReceiver.decodedIRData.command) {
      case RemoteButtons::ePwr:  // Toggle state between JSON and IR standby modes
        state = eStandbyRC;
        DEBUG_PRINTLN("Cycle state: Switching to RC mode");
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
      case RemoteButtons::eZero:          // move belt forwards
      case RemoteButtons::eOne:          // move belt forwards
        // For each of these cases, setup the move according to the button press
        move = setupMoveFromIRCommand((RemoteButtons)IrReceiver.decodedIRData.command);
        moveQueue->push(move);
        executeMoveSequence(moveQueue);
        break;
      case RemoteButtons::eFuncStop:  // Enter adjustment mode
        state = eAdjustmentMode;
        gCurrentAdjustmentSubMode = eNotAdjusting;  // Reset to not adjusting
        DEBUG_PRINTLN("Entering adjustment mode");
        break;
      // Add additional case handlers as needed
      default:
        DEBUG_PRINTLN("IR Command not handled.");
        break;
    }
    IrReceiver.resume();
    delay(100);  //debounce
  }
}

void standbyRC(States& state) {
  DEBUG_PRINT("STANDBY RC... <");
  DEBUG_PRINT(millis() / 1000.0);
  DEBUG_PRINTLN(">");

  if (IrReceiver.decode()) {

    gLastRCCommandTime = millis();
    float wheelSpeeds[cNumberOfWheels];

    switch ((RemoteButtons)IrReceiver.decodedIRData.command) {
      case RemoteButtons::ePwr:  // Toggle state between JSON and IR standby modes
        state = eStandbyJSON;
        DEBUG_PRINTLN("Cycle state: Switching to JSON mode");
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
  } else {
    if (millis() - gLastRCCommandTime > cRCCommandTimeout) {
      gMecanumMotors.setSpeeds(0, 0, 0, 0);  // Set speeds to 0 after timeout
      gL2Motors.setSpeeds(0, 0);
      // reset lastCommandTime here to prevent repeatedly setting speeds to 0
      gLastRCCommandTime = millis();
    }
  }
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
    case RemoteButtons::eTwo:  // move platform up
      move.moveType = MoveType::eScissor;
      move.params.scissorParams.direction = 1;
      break;
    case RemoteButtons::eEight:  // move platform down
      move.moveType = MoveType::eScissor;
      move.params.scissorParams.direction = 0;
      break;
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
      move.params.calibrationParams.duration = 3000; // make this changable in the configuration mode
      break;
    case RemoteButtons::eOne:
      move.moveType = MoveType::eLineFollow;
      move.params.linefollowParams.speed = gDriveSpeed;
      move.params.linefollowParams.stopDistance = 10; // make this changable in the configuration mode
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
    //Commented because I'm using serial and its getting annoying
    //Serial.print(F("deserializeJson() failed: "));
    //Serial.println(error.f_str());
    return;
  }
}

void executeAdjustmentMode(States& state) {
  if (IrReceiver.decode()) {
    switch ((RemoteButtons)IrReceiver.decodedIRData.command) {
      case RemoteButtons::eZero:
        gCurrentAdjustmentSubMode = eAdjustingDriveSpeed;
        DEBUG_PRINTLN("Selected gDriveSpeed for adjustment.");
        break;
      case RemoteButtons::eOne:
        gCurrentAdjustmentSubMode = eAdjustingRemoteControlDuration;
        DEBUG_PRINTLN("Selected gRemoteControlDuration for adjustment.");
        break;
      case RemoteButtons::eVolPlus:
        switch (gCurrentAdjustmentSubMode) {
          case eAdjustingDriveSpeed:
            gDriveSpeed = std::min(400, gDriveSpeed + 100);
            DEBUG_PRINT("gDriveSpeed increased to: ");
            DEBUG_PRINTLN(gDriveSpeed);
            break;
          case eAdjustingRemoteControlDuration:
            gRemoteControlDuration = std::min(10000, gRemoteControlDuration + 500);
            DEBUG_PRINT("gRemoteControlDuration increased to: ");
            DEBUG_PRINTLN(gRemoteControlDuration);
            break;
        }
        break;
      case RemoteButtons::eVolMinus:
        switch (gCurrentAdjustmentSubMode) {
          case eAdjustingDriveSpeed:
            gDriveSpeed = std::max(0, gDriveSpeed - 100);
            DEBUG_PRINT("gDriveSpeed decreased to: ");
            DEBUG_PRINTLN(gDriveSpeed);
            break;
          case eAdjustingRemoteControlDuration:
            gRemoteControlDuration = std::max(0, gRemoteControlDuration - 500);
            DEBUG_PRINT("gRemoteControlDuration decreased to: ");
            DEBUG_PRINTLN(gRemoteControlDuration);
            break;
        }
        break;
      case RemoteButtons::eFuncStop:
        state = eStandbyIR;                         // Go back to standby IR mode
        gCurrentAdjustmentSubMode = eNotAdjusting;  // Reset adjustment mode
        DEBUG_PRINTLN("Exiting adjustment mode.");
        break;
    }
    IrReceiver.resume();
    delay(100);  //debounce
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

  switch (nextMove.params.freedriveParams.direction) {
    case eForwards:
      gWheelbase->computeWheelSpeeds(0, 10, 0, wheelSpeeds);
      runWheelMotorsWithBlockingDelay(delayTime, wheelSpeeds);
      break;
    case eLeft:
      gWheelbase->computeWheelSpeeds(-10, 0, 0, wheelSpeeds);
      runWheelMotorsWithBlockingDelay(delayTime, wheelSpeeds);
      break;
    case eBackwards:
      gWheelbase->computeWheelSpeeds(0, -10, 0, wheelSpeeds);
      runWheelMotorsWithBlockingDelay(delayTime, wheelSpeeds);
      break;
    case eRight:
      gWheelbase->computeWheelSpeeds(10, 0, 0, wheelSpeeds);
      runWheelMotorsWithBlockingDelay(delayTime, wheelSpeeds);
      break;
    case eCCW:
      gWheelbase->computeWheelSpeeds(0, 0, 1.059, wheelSpeeds);
      runWheelMotorsWithBlockingDelay(delayTime, wheelSpeeds);
      break;
    case eCW:
      gWheelbase->computeWheelSpeeds(0, 0, -1.059, wheelSpeeds);
      runWheelMotorsWithBlockingDelay(delayTime, wheelSpeeds);
      break;
    default:
      DEBUG_PRINTLN("Unexpected input in direction switch for freedrive.");
      break;
  }
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
      uint16_t position = gQtr.readLineBlack(sensorValues);
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
  if (targetWheelSpeeds) {
    DEBUG_PRINTLN("Running wheel motors with blocking delay and speed ramp.");

    // Map target wheel speeds from their current values to a scale suitable for the motor drivers before ramping.
    float mappedSpeeds[cNumberOfWheels];
    memcpy(mappedSpeeds, targetWheelSpeeds, sizeof(mappedSpeeds));  // Copy to preserve original target speeds
    mapWheelSpeeds(mappedSpeeds, gDriveSpeed);                      // map to global drive speed

    // Ramp speeds up to mapped target values over a period (e.g., 200 milliseconds)
    rampMotorSpeed(mappedSpeeds, 200, true);  // true ramps up

    // Wait for the specified delay time after ramping to the target speed.
    delay(delayTime);

    // Optionally, smoothly ramp down to 0 for a soft stop.
    rampMotorSpeed(mappedSpeeds, 200, false);  // false ramps down

    DEBUG_PRINTLN("Motors stopped.");
  } else {
    DEBUG_PRINTLN("Error: targetWheelSpeeds is null.");
  }
}

void runWheelMotorsDirectly(float* targetWheelSpeeds) {
  if (targetWheelSpeeds) {

    // Map target wheel speeds from their current values to a scale suitable for the motor drivers before ramping.
    float mappedSpeeds[cNumberOfWheels];
    mapWheelSpeeds(targetWheelSpeeds, gDriveSpeed);  // map to global drive speed
    gMecanumMotors.setSpeeds(targetWheelSpeeds[0], -targetWheelSpeeds[1], targetWheelSpeeds[2], -targetWheelSpeeds[3]);

  } else {
    DEBUG_PRINTLN("Error: targetWheelSpeeds is null.");
  }
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
        memcpy(currentSpeed, targetWheelSpeeds, sizeof(currentSpeed));
        gMecanumMotors.setSpeeds(currentSpeed[0], -currentSpeed[1], currentSpeed[2], -currentSpeed[3]);
      } else {
        gMecanumMotors.setSpeeds(0, 0, 0, 0);
      }
      break;  // Exit loop
    } else {
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
    }

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

void setPinModes() {
  pinMode(cTopLimitSwitchPin, INPUT);
  pinMode(cBottomLimitSwitchPin, INPUT);
}
