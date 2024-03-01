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
#include <IRremote.h>

// Global variables :(
// QUANTITIES
const int cNumberOfWheels = 4;
const uint8_t cSensorCount = 8;
// PARAMETERS
const int cFilterWindowSize = 20;
// PINS
const int cDistPin1 = A4;              // Left IR rangefinder sensor
const int cDistPin2 = A5;              // Right IR rangefinder sensor
const int cTopLimitSwitchPin = 53;     // Replace XX with the actual pin number
const int cBottomLimitSwitchPin = 52;  // Replace YY with the actual pin number
const int cIrRecvPin = 11;             // IR Reciever

// Sensor globals
uint16_t sensorValues[cSensorCount];
std::queue<float> gDistSensor1Readings;
std::queue<float> gDistSensor2Readings;
QTRSensors gQtr;

// Motor globals
DualTB9051FTGMotorShieldMod3230 gMecanumMotors;
L298NMotorDriverMega gL2Motors(5, 34, 32, 6, 33, 35);
Wheelbase* gWheelbase = new Wheelbase(5.0625, 4.386, 2.559);

// IR reciever globals
IRrecv gIrReciever(cIrRecvPin);
decode_results gIrDecodeResults;

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
  gIrReciever.enableIRIn();  // Start the receiver

  // ...
  setPinModes();

  loop(doc);
}

void loop(JsonDocument& doc) {
  // Control flow globals :(
  std::queue<Move>* moveQueue = new std::queue<Move>();
  States state = eStandby;

  // LOOP BEGINS
  // -------------------------------------------------
  while (true) {
    switch (state) {
      case eStandby:
        waitingToStart(doc, moveQueue, state);
        break;
      case eMoving:
        executeMoveSequence(moveQueue);
        // Done executing moves
        state = eStandby;
        break;
    }
  }
  // -------------------------------------------------
}

void waitingToStart(JsonDocument& doc, std::queue<Move>* moveQueue, States& state) {
  DEBUG_PRINT("STANDBY... <");
  DEBUG_PRINT(millis() / 1000.0);
  DEBUG_PRINTLN(">");
  // Check serial for JSON packet to decide
  read_serial(doc);
  if (doc.isNull()) {
    return;
  } else if (doc.containsKey("g")) {
    parseJsonIntoQueue(moveQueue, doc);
    state = eMoving;
  }
  // Check IR Reciever for IR signal to decode
  if (gIrReciever.decode(&gIrDecodeResults)) {
    // note: FFFFFF is a repeat command. You get it while you hold a button down.
    Serial2.println(gIrDecodeResults.value, HEX);
    switch ((RemoteButtons)gIrDecodeResults.value) {
      case RemoteButtons::ePwr:  // PWR
        // Handle PWR button press
        break;
      case RemoteButtons::eVolPlus:  // VOL+
        // Handle VOL+ button press
        break;
      case RemoteButtons::eFuncStop:  // FUNC/STOP
        // Handle FUNC/STOP button press
        break;
      case RemoteButtons::eBack:  // |<<
        // Handle |<< button press
        break;
      case RemoteButtons::eForward:  // >|
        // Handle >| button press
        break;
      case RemoteButtons::eFastForward:  // >>|
        // Handle >>| button press
        break;
      case RemoteButtons::eDown:  // DOWN
        // Handle DOWN button press
        break;
      case RemoteButtons::eVolMinus:  // VOL-
        // Handle VOL- button press
        break;
      case RemoteButtons::eUp:  // UP
        // Handle UP button press
        break;
      case RemoteButtons::eZero:  // 0
        // Handle 0 button press
        break;
      case RemoteButtons::eEq:  // EQ
        // Handle EQ button press
        break;
      case RemoteButtons::eStRept:  // ST/REPT
        // Handle ST/REPT button press
        break;
      case RemoteButtons::eOne:  // 1
        // Handle 1 button press
        break;
      case RemoteButtons::eTwo:  // 2
        // Handle 2 button press
        break;
      case RemoteButtons::eThree:  // 3
        // Handle 3 button press
        break;
      case RemoteButtons::eFour:  // 4
        // Handle 4 button press
        break;
      case RemoteButtons::eFive:  // 5
        // Handle 5 button press
        break;
      case RemoteButtons::eSix:  // 6
        // Handle 6 button press
        break;
      case RemoteButtons::eSeven:  // 7
        // Handle 7 button press
        break;
      case RemoteButtons::eEight:  // 8
        // Handle 8 button press
        break;
      case RemoteButtons::eNine:  // 9
        // Handle 9 button press
        break;
      default:
        // Handle unknown or repeat command
        break;
    }
    gIrReciever.resume();  // Receive  the next value
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
      runMotorsWithBlockingDelay(delayTime, wheelSpeeds);
      break;
    case eLeft:
      gWheelbase->computeWheelSpeeds(-10, 0, 0, wheelSpeeds);
      runMotorsWithBlockingDelay(delayTime, wheelSpeeds);
      break;
    case eBackwards:
      gWheelbase->computeWheelSpeeds(0, -10, 0, wheelSpeeds);
      runMotorsWithBlockingDelay(delayTime, wheelSpeeds);
      break;
    case eRight:
      gWheelbase->computeWheelSpeeds(10, 0, 0, wheelSpeeds);
      runMotorsWithBlockingDelay(delayTime, wheelSpeeds);
      break;
    case eCCW:
      gWheelbase->computeWheelSpeeds(0, 0, 1.059, wheelSpeeds);
      runMotorsWithBlockingDelay(delayTime, wheelSpeeds);
      break;
    case eCW:
      gWheelbase->computeWheelSpeeds(0, 0, -1.059, wheelSpeeds);
      runMotorsWithBlockingDelay(delayTime, wheelSpeeds);
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
    gL2Motors.setM2Speed(100);
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
    gL2Motors.setM2Speed(-100);
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

void runMotorsWithBlockingDelay(int delayTime, float* targetWheelSpeeds) {
  if (targetWheelSpeeds) {
    DEBUG_PRINTLN("Running wheel motors with blocking delay and speed ramp.");

    // Map target wheel speeds from their current values to a scale suitable for the motor drivers before ramping.
    float mappedSpeeds[cNumberOfWheels];
    memcpy(mappedSpeeds, targetWheelSpeeds, sizeof(mappedSpeeds));  // Copy to preserve original target speeds
    mapWheelSpeeds(mappedSpeeds, 200);                              // hard coded value shoud be changed at some point

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
