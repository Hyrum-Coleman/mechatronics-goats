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
//#include <DualTB9051FTGMotorShieldBarebones.h>
#include <DualTB9051FTGMotorShieldMod3230.h>
#include <L298NMotorDriverMega.h>
#include <queue>
#include "Wheelbase.h"
//#include "L298NMotorDriver.h"


// Global variables :(
const int cNumberOfWheels = 4;

DualTB9051FTGMotorShieldMod3230 gMecanumMotors;
L298NMotorDriverMega gSmolMotors(5, 32, 33, 6, 34, 35);
//L298NMotorDriver small_motors(34,32,33,35,5,6);

Wheelbase* gWheelbase = new Wheelbase(5.0625, 4.386, 2.559);

// Structs and enums
struct Move {
  unsigned short direction;
  unsigned long time;
};

enum States {
  eDriving = 0,
  eWaitingToStart = 1,
};

enum Directions {
  eForwards = 1,
  eLeft = 2,
  eBackwards = 3,
  eRight = 4,
  eCCW = 5,
  eCW = 6,
  eLift = 7,
  eBelt = 8,
};

JsonDocument gDoc;

void setup() {
  // init();  // Initialize board
  Serial.begin(9600);
  Serial2.begin(9600);
  Serial2.setTimeout(10000);

  // JsonDocument doc;

  gMecanumMotors.init();
  gMecanumMotors.enableDrivers();

  gSmolMotors.init();
  //small_motors.init();

  // loop(doc);
}

void loop() {
  std::queue<Move>* moveQueue = new std::queue<Move>();
  States state = eWaitingToStart;

  // LOOP BEGINS
  while (true) {
    switch (state) {
      case eWaitingToStart:
        DEBUG_PRINTLN("WAITING TO START");
        read_serial(gDoc);
        if (gDoc.isNull()) {
          continue;
        } else if (gDoc.containsKey("d")) {
          DEBUG_PRINTLN("ABOUT TO DESERIALIZE D KEY");
          deserializeDKeyIntoQueue(moveQueue, gDoc);
          DEBUG_PRINTLN("DONE DESERIALIZING");
          state = eDriving;
        }
        break;
      case eDriving:
        DEBUG_PRINTLN("JUST ENTERED DRIVING STATE");
        if (moveQueue->empty()) {
          DEBUG_PRINTLN("MOVEQUEUE EMPTY.");
          state = eWaitingToStart;  // Go back to waiting state if queue is empty
          DEBUG_PRINTLN("STATE SET TO WAITING TO START!");
          break;
        }
        driving_logic(moveQueue);
        break;
    }
  }
}

void deserializeDKeyIntoQueue(std::queue<Move>* moveQueue, JsonDocument& doc) {
  for (JsonObject obj : doc["d"].as<JsonArray>()) {
    Move currentMove;
    currentMove.direction = obj["a"];
    currentMove.time = obj["t"];
    moveQueue->push(currentMove);

    DEBUG_PRINT("Pushed Move - Direction: ");
    DEBUG_PRINT(currentMove.direction);
    DEBUG_PRINT(", Time: ");
    DEBUG_PRINTLN(currentMove.time);
  }

  DEBUG_PRINT("Total Moves in Queue: ");
  DEBUG_PRINTLN(moveQueue->size());
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

void driving_logic(std::queue<Move>* moveQueue) {
  float wheelSpeeds[cNumberOfWheels];  // Initialize motor speeds

  Move nextMove = getNextMoveFromQueue(moveQueue);
  int delayTime = nextMove.time;
  int motorMax = 400;
  switch ((Directions)nextMove.direction) {
    DEBUG_PRINT("NEXTMOVE.DIRECTION: ");
    DEBUG_PRINTLN(nextMove.direction);

    case eForwards:  // forwards
      DEBUG_PRINTLN("Driving: Forwards");
      gWheelbase->computeWheelSpeeds(0, 10, 0, wheelSpeeds);
      runMotorsWithBlockingDelay(delayTime, wheelSpeeds, motorMax, false);
      break;
    case eLeft:  // left
      DEBUG_PRINTLN("Driving: Left");
      gWheelbase->computeWheelSpeeds(-10, 0, 0, wheelSpeeds);
      runMotorsWithBlockingDelay(delayTime, wheelSpeeds, motorMax, false);
      break;
    case eBackwards:  // backwards
      DEBUG_PRINTLN("Driving: Backwards");
      gWheelbase->computeWheelSpeeds(0, -10, 0, wheelSpeeds);
      runMotorsWithBlockingDelay(delayTime, wheelSpeeds, motorMax, false);
      break;
    case eRight:  // right
      DEBUG_PRINTLN("Driving: Right");
      gWheelbase->computeWheelSpeeds(10, 0, 0, wheelSpeeds);
      runMotorsWithBlockingDelay(delayTime, wheelSpeeds, motorMax, false);
      break;
    case eCCW:  // rotate ccw
      DEBUG_PRINTLN("Driving: rotate ccw");
      // Rotations are more sensitive. I hand calculated 1.059 to match our map scale
      gWheelbase->computeWheelSpeeds(0, 0, 1.059, wheelSpeeds);
      runMotorsWithBlockingDelay(delayTime, wheelSpeeds, motorMax, false);
      break;
    case eCW:  // rotate cw
      DEBUG_PRINTLN("Driving: rotate cw");
      // Rotations are more sensitive. I hand calculated 1.059 to match our map scale
      gWheelbase->computeWheelSpeeds(0, 0, -1.059, wheelSpeeds);
      runMotorsWithBlockingDelay(delayTime, wheelSpeeds, motorMax, false);
      break;
    case eLift:  // lift motor
      DEBUG_PRINTLN("Driving: lift motor");
      runMotorsWithBlockingDelay(delayTime, nullptr, motorMax, true);
      break;
    case eBelt:  // belt motor
      DEBUG_PRINTLN("Driving: belt motor");
      runMotorsWithBlockingDelay(delayTime, nullptr, motorMax, false);
      break;
    default:
      DEBUG_PRINT("Unexpected input in direction switch: ");
      DEBUG_PRINTLN(nextMove.direction);
      break;
  }
}

Move getNextMoveFromQueue(std::queue<Move>* queueToPopFrom) {
  Move retMove = queueToPopFrom->front();
  //Move retMove = queueToPopFrom->back();

  queueToPopFrom->pop();

  return retMove;
}

void runMotorsWithBlockingDelay(int delayTime, float* wheelSpeeds, unsigned long speed, bool lift_motor) {

  // if motorSpeeds is accessesed outside this if, a segfault will be issued :trollface:
  if (wheelSpeeds) {
    DEBUG_PRINT("Wheel Speeds before mapping: ");
    for (int i = 0; i < cNumberOfWheels; i++) {
      DEBUG_PRINT(wheelSpeeds[i]);
      DEBUG_PRINT(" ");  // Space between values for readability
    }

    DEBUG_PRINTLN("");                    // New line after printing all speeds
    mapWheelSpeeds(wheelSpeeds, speed);  // mutates motorSpeeds

    DEBUG_PRINT("Wheel Speeds after mapping: ");
    for (int i = 0; i < cNumberOfWheels; i++) {
      DEBUG_PRINT(wheelSpeeds[i]);
      DEBUG_PRINT(" ");  // Space between values for readability
    }
    DEBUG_PRINTLN("");  // New line after printing all speeds

    // Negative signs are to account for polarity of motors. Motors are wired to positive goes to A and negative to B
    gMecanumMotors.setSpeeds(wheelSpeeds[0], -wheelSpeeds[1], wheelSpeeds[2], -wheelSpeeds[3]);

  } else {
    if (lift_motor) {
      DEBUG_PRINT("Setting M1 speed to ");
      DEBUG_PRINTLN(speed);

      gSmolMotors.setM1Speed(200);

      delay(delayTime);
      gSmolMotors.flipM1(false);

      //small_motors.setMotorA(255, true);
    } else {
      DEBUG_PRINT("Setting M2 speed to ");
      DEBUG_PRINTLN(speed);

      gSmolMotors.setM2Speed(200);
      delay(delayTime);
      gSmolMotors.setM2Speed(-200);
      //small_motors.setMotorB(255, true);
    }
  }

  delay(delayTime);

  // turns off motors after delay
  if (wheelSpeeds) {
    gMecanumMotors.setSpeeds(0, 0, 0, 0);
  } else {
    if (lift_motor) {
      gSmolMotors.setM1Brake(0);
      //small_motors.stopMotorA();
    } else {
      gSmolMotors.setM2Brake(0);
      //small_motors.stopMotorB();
    }
  }
}

void mapWheelSpeeds(float* wheelSpeeds, unsigned long maxSpeed) {

  for (int i = 0; i < cNumberOfWheels; i++) {
    wheelSpeeds[i] = map(wheelSpeeds[i], -3.91, 3.91, -1 * maxSpeed, maxSpeed);
  }
}
