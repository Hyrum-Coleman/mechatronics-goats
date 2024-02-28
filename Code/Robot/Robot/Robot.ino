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

// Global variables :(
// QUANTITIES
const int cNumberOfWheels = 4;
const uint8_t SensorCount = 8;
// PARAMETERS
const int MA_WINDOW_SIZE = 5;
// PINS
const int distPin1 = A4;              // Left IR rangefinder sensor
const int distPin2 = A5;              // Right IR rangefinder sensor
const int topLimitSwitchPin = 53;     // Replace XX with the actual pin number
const int bottomLimitSwitchPin = 52;  // Replace YY with the actual pin number
// DATASTRUCTURES
uint16_t sensorValues[SensorCount];
std::queue<float> distSensor1Readings;
std::queue<float> distSensor2Readings;

DualTB9051FTGMotorShieldMod3230 gMecanumMotors;
L298NMotorDriverMega gL2Motors(5, 34, 32, 6, 33, 35);
Wheelbase* gWheelbase = new Wheelbase(5.0625, 4.386, 2.559);
QTRSensors qtr;

// Structs and enums
enum MoveType {
  eFreeDrive = 0,
  eLineFollow = 1,
  eScissor = 2,
  eBelt = 3,
  eCalibrate = 4,
};

// Onion for move-specific parameters
union MoveParameters {
  struct {
    unsigned short direction;  // which way to drive (will be [x,y,theta] in the future)
    unsigned long duration;    // how far to go (will be distance not time in the future)
  } freedriveParams;           // For eFreeDrive

  struct {
    unsigned long stopDistance;  // how far to stop away from obstacle when line following
    int speed;
  } linefollowParams;  // For eLineFollow

  struct {
    bool direction;  // which way to move the scissor jack. 1 for up 0 for down.
  } scissorParams;   // For eScissor

  struct {
    bool direction;          // which way to drive the belt
    unsigned long duration;  // how long to drive the belt
  } beltParams;              // For belt

  struct {
    unsigned long duration;  // How long to calibrate line follower
  } calibrationParams;       // For calibrating sensors
};

struct Move {
  MoveType moveType;
  MoveParameters params;
};

enum States {
  eMoving = 0,
  eWaitingToStart = 1,
};

enum Directions {
  eForwards = 1,
  eLeft = 2,
  eBackwards = 3,
  eRight = 4,
  eCCW = 5,
  eCW = 6,
};

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
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){
                      36, 38, 40, 42, 43, 41, 39, 37 },
                    SensorCount);

  // ...
  setPinModes();

  loop(doc);
}

void loop(JsonDocument& doc) {
  std::queue<Move>* moveQueue = new std::queue<Move>();
  States state = eWaitingToStart;

  // LOOP BEGINS
  // -------------------------------------------------
  while (true) {
    switch (state) {
      case eWaitingToStart:
        DEBUG_PRINTLN("WAITING TO START");
        read_serial(doc);
        if (doc.isNull()) {
          continue;
        } else if (doc.containsKey("g")) {
          parseJsonIntoQueue(moveQueue, doc);
          state = eMoving;
        }
        break;
      case eMoving:
        executeMoveSequence(moveQueue);
        // Done executing moves
        state = eWaitingToStart;
        break;
    }
  }
  // -------------------------------------------------
}

void parseJsonIntoQueue(std::queue<Move>* moveQueue, JsonDocument& doc) {
  for (JsonObject obj : doc["g"].as<JsonArray>()) {  // g for go
    Move currentMove;

    // Populate move structs params based on move type
    String moveType = obj["type"].as<String>();
    if (moveType == "freedrive") {
      currentMove.moveType = eFreeDrive;
      currentMove.params.freedriveParams.direction = obj["direction"];
      currentMove.params.freedriveParams.duration = obj["duration"];
    } else if (moveType == "linefollow") {
      currentMove.moveType = eLineFollow;
      currentMove.params.linefollowParams.stopDistance = obj["stopDistance"];
      currentMove.params.linefollowParams.speed = obj["speed"];
    } else if (moveType == "scissor") {
      currentMove.moveType = eScissor;
      currentMove.params.scissorParams.direction = obj["direction"];
    } else if (moveType == "belt") {
      currentMove.moveType = eBelt;
      currentMove.params.beltParams.direction = obj["direction"];
      currentMove.params.beltParams.duration = obj["duration"];
    } else if (moveType == "calibrate") {
      currentMove.moveType = eCalibrate;
      currentMove.params.calibrationParams.duration = obj["duration"];
    } else {
      // If move unknown,
      continue;  // Skip this move
    }
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
    switch ((MoveType)nextMove.moveType) {
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

// todo: get this working. currently stopping because distance threshold so the distance sensors are getting noisy values
void executeLineFollow(Move nextMove) {
  float targetDistance = nextMove.params.linefollowParams.stopDistance;
  int baseSpeed = nextMove.params.linefollowParams.speed;

  while (true) {
    // Perform line following logic
    uint16_t position = qtr.readLineBlack(sensorValues);
    int error = position - 3500;  // Center is 3500 for 8 sensors
    double Kp = 1.0 / 20.0;

    int leftSpeed = baseSpeed + (Kp * error);
    int rightSpeed = baseSpeed - (Kp * error);

    // Set motor speeds based on line position
    gMecanumMotors.setSpeeds(leftSpeed, -rightSpeed, leftSpeed, -rightSpeed);

    // Check distance to wall using distance sensors
    float distanceLeft = pollRangefinder(distPin1);
    float distanceRight = pollRangefinder(distPin2);
    // Check filtered distance to wall using distance sensors
    //float distanceLeft = pollRangefinderWithSMA(distPin1, distSensor1Readings); // distSensor1Readings is a global
    //float distanceRight = pollRangefinderWithSMA(distPin2, distSensor2Readings); // distSensor2Readings is a global


    DEBUG_PRINTLN(distanceLeft);
    DEBUG_PRINTLN(distanceRight);

    // If close enough to the wall, stop
    if (distanceLeft <= targetDistance || distanceRight <= targetDistance) {
      gMecanumMotors.setSpeeds(0, 0, 0, 0);  // Stop the robot
      break;                                 // Exit the loop
    }

    delay(100);  // Short delay to avoid overly frequent sensor readings and motor updates
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
    while (digitalRead(topLimitSwitchPin) == HIGH) {
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
    while (digitalRead(bottomLimitSwitchPin) == HIGH) {
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

    // Ramp speeds up to mapped target values over a period (e.g., 1000 milliseconds)
    rampMotorSpeed(mappedSpeeds, 1000);

    // Wait for the specified delay time after ramping to the target speed.
    delay(delayTime);

    // Optionally, smoothly ramp down to 0 for a soft stop.
    float stopSpeeds[cNumberOfWheels] = { 0, 0, 0, 0 };
    rampMotorSpeed(stopSpeeds, 1000);  // Smooth ramp down

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
    qtr.calibrate();
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
void rampMotorSpeed(float* targetWheelSpeeds, int rampDuration) {
  unsigned long rampStartTime = millis();
  unsigned long currentTime;
  float currentSpeed[cNumberOfWheels] = { 0, 0, 0, 0 };  // Start speeds at 0

  while (true) {
    currentTime = millis() - rampStartTime;
    float rampProgress = (float)currentTime / (float)rampDuration;

    if (rampProgress >= 1.0) {
      // If ramp is complete, ensure target speed is set
      memcpy(currentSpeed, targetWheelSpeeds, sizeof(currentSpeed));
      gMecanumMotors.setSpeeds(currentSpeed[0], -currentSpeed[1], currentSpeed[2], -currentSpeed[3]);
      break;  // Exit loop
    } else {
      // Calculate and set intermediate speeds
      for (int i = 0; i < cNumberOfWheels; i++) {
        currentSpeed[i] = targetWheelSpeeds[i] * rampProgress;
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
  if (readingsQueue.size() >= MA_WINDOW_SIZE) {
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
  pinMode(topLimitSwitchPin, INPUT);
  pinMode(bottomLimitSwitchPin, INPUT);
}
