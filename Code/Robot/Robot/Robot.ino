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
// Custom versions of libraries to debug (kinda useless)
//#include "L298NMotorDriver.h"
//#include <DualTB9051FTGMotorShieldBarebones.h>

// Global variables :(
const int cNumberOfWheels = 4;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
const int distPin1 = A4; // Left IR rangefinder sensor
const int distPin2 = A5; // Right IR rangefinder sensor
const int topLimitSwitchPin = 53; // Replace XX with the actual pin number
const int bottomLimitSwitchPin = 52; // Replace YY with the actual pin number

DualTB9051FTGMotorShieldMod3230 gMecanumMotors;
L298NMotorDriverMega gL2Motors(5, 32, 33, 6, 34, 35);
Wheelbase* gWheelbase = new Wheelbase(5.0625, 4.386, 2.559);
QTRSensors qtr;

// Structs and enums
enum MoveType {
  eFreeDrive = 0,
  eLineFollow = 1,
  eScissor = 2,
  eBelt = 3,
};

// Onion for move-specific parameters
union MoveParameters {
  struct {
    unsigned short direction; // which way to drive (will be [x,y,theta] in the future)
    unsigned long duration; // how far to go (will be distance not time in the future)
  } freedriveParams; // For eFreeDrive

  struct {
    unsigned long stopDistance; // how far to stop away from obstacle when line following
  } linefollowParams; // For eLineFollow

  struct {
    bool direction; // which way to move the scissor jack. 1 for up 0 for down.
  } scissorParams; // For eScissor

  struct {
    bool direction; // which way to drive the belt
    unsigned long duration; // how long to drive the belt
  } beltParams; // For belt
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
  init();  // Initialize board
  Serial.begin(9600);
  Serial2.begin(9600);
  Serial2.setTimeout(10000);

  JsonDocument doc;

  gMecanumMotors.init();
  gMecanumMotors.enableDrivers();

  gL2Motors.init();
  //small_motors.init();

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

// New JSON format.
// Needs to be abbreviated to save bytes.
// Suggest: x for type, d for direction, t for duration, f l s and b fir types.
/*
{
  "g": [
    {
      "type": "freedrive",
      "direction": 1,
      "duration": 5000
    },
    {
      "type": "linefollow",
      "stopDistance": 100
    },
    {
      "type": "scissor",
      "direction": 1, // 1 or 0 for top and bottom
    },
    {
      "type": "belt",
      "duration": 2000
    }
  ]
}
*/
void parseJsonIntoQueue(std::queue<Move>* moveQueue, JsonDocument& doc) {
    for (JsonObject obj : doc["g"].as<JsonArray>()) { // g for go
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
        } else if (moveType == "scissor") {
            currentMove.moveType = eScissor;
            currentMove.params.scissorParams.direction = obj["direction"];
        } else if (moveType == "belt") {
            currentMove.moveType = eBelt;
            currentMove.params.beltParams.duration = obj["duration"];
        } else {
            // If move unknown,
            continue; // Skip this move
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
  while (!moveQueue->empty()) { // Loop until the queue is empty
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
      default:
        DEBUG_PRINT("Unexpected moveType: ");
        DEBUG_PRINTLN(nextMove.moveType);
        break;
    }
  }
}

void executeFreeDrive(Move nextMove) {
  float wheelSpeeds[cNumberOfWheels]; // Initialize motor speeds
  int delayTime = nextMove.params.freedriveParams.duration;

  switch ((Directions)nextMove.params.freedriveParams.direction) {
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

  while (true) {
        // Perform line following logic
        uint16_t position = qtr.readLineBlack(sensorValues);
        int error = position - 3500; // Center is 3500 for 8 sensors
        double Kp = 1.0 / 20.0;

        int baseSpeed = 200; // Base speed for each motor
        int leftSpeed = baseSpeed - (Kp * error);
        int rightSpeed = baseSpeed + (Kp * error);

        // Set motor speeds based on line position
        gMecanumMotors.setSpeeds(leftSpeed, -rightSpeed, leftSpeed, -rightSpeed);

        // Check distance to wall using distance sensors
        float distanceLeft = pollRangefinder(distPin1);
        float distanceRight = pollRangefinder(distPin2);

        // If close enough to the wall, stop
        if (distanceLeft <= targetDistance || distanceRight <= targetDistance) {
            gMecanumMotors.setSpeeds(0, 0, 0, 0); // Stop the robot
            break; // Exit the loop
        }

        delay(100); // Short delay to avoid overly frequent sensor readings and motor updates
    }
}

// NOTE: THE DIRECTION OF THE MOTOR TO GO UP VS DOWN MAY NEED TO BE CHANGED!!!
void executeScissor(Move nextMove) {
    unsigned long targetHeight = nextMove.params.scissorParams.direction;

    if (targetHeight == 1) {
        // Move towards the top limit switch
        gL2Motors.setM2Speed(100);
        while(digitalRead(topLimitSwitchPin) == LOW) {
            // Keep moving until the top limit switch is triggered
            delay(10);
        }
    } else if (targetHeight == 0) {
        // Move towards the bottom limit switch
        gL2Motors.setM2Speed(-100);
        while(digitalRead(bottomLimitSwitchPin) == LOW) {
            // Keep moving until the bottom limit switch is triggered
            delay(10); // Small delay to prevent too rapid polling
        }
    }

    gL2Motors.setM2Speed(0); // Stop the motor once the limit switch is reached
}

void executeBelt(Move nextMove) {
    unsigned long duration = nextMove.params.beltParams.duration; // Duration in milliseconds
    bool direction = nextMove.params.beltParams.direction; // Direction (1 is forward, 0 is backward)

    // Determine speed based on direction
    int speed = direction ? 400 : -400; // Assume positive speed for forward, negative for backward

    gL2Motors.setM1Speed(speed); // Set speed and direction
    delay(duration); // Run for specified duration
    gL2Motors.setM1Speed(0); // Stop the belt
}

Move getNextMoveFromQueue(std::queue<Move>* queueToPopFrom) {
  Move retMove = queueToPopFrom->front();
  //Move retMove = queueToPopFrom->back();

  queueToPopFrom->pop();

  return retMove;
}

void runMotorsWithBlockingDelay(int delayTime, float* wheelSpeeds) {
  if (wheelSpeeds) {
    DEBUG_PRINTLN("Running wheel motors with blocking delay.");

    // Map wheel speeds from their current values to a scale suitable for the motor drivers.
    mapWheelSpeeds(wheelSpeeds, 400); // changed this back to hardcoded value. should probably be a pound define.

    // Set the motor speeds.
    gMecanumMotors.setSpeeds(wheelSpeeds[0], -wheelSpeeds[1], wheelSpeeds[2], -wheelSpeeds[3]);

    // Wait for the specified delay time.
    delay(delayTime);

    // Turn off motors after the delay.
    gMecanumMotors.setSpeeds(0, 0, 0, 0);

    DEBUG_PRINTLN("Motors stopped.");
  } else {
    // Log an error if wheelSpeeds is null.
    DEBUG_PRINTLN("Error: wheelSpeeds is null.");
  }
}

void mapWheelSpeeds(float* wheelSpeeds, unsigned long maxSpeed) {

  for (int i = 0; i < cNumberOfWheels; i++) {
    wheelSpeeds[i] = map(wheelSpeeds[i], -3.91, 3.91, -1 * maxSpeed, maxSpeed);
  }
}

float pollRangefinder(int pin) {
  int sensorValue = analogRead(pin);
  float voltage = sensorValue * (5.0 / 1023.0);
  float distance = 33.9 - 69.5 * voltage + 62.3 * pow(voltage, 2) - 25.4 * pow(voltage, 3) + 3.83 * pow(voltage, 4);
  return distance;
}

void setPinModes(){
  pinMode(topLimitSwitchPin, INPUT);
  pinMode(bottomLimitSwitchPin, INPUT);
}
