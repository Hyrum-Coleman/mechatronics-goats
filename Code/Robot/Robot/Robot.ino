// Enable or disable debug prints
#define DEBUG_PRINTS_ENABLED true

#if DEBUG_PRINTS_ENABLED
#define debugPrint(x) Serial2.print(x)
#define debugPrintln(x) Serial2.println(x)
#else
#define debugPrint(x)
#define debugPrintln(x)
#endif

// Include dependencies
#include <Arduino.h>
#include <ArduinoSTL.h>
#include <ArduinoJson.h>
#include <StreamUtils.h>
#include "DualTB9051FTGMotorShieldMod3230.h"
#include <L298NMotorDriverMega.h>
#include <queue>
#include "Wheelbase.h"


// Global variables :(
const int NUMBER_OF_WHEELS = 4;

DualTB9051FTGMotorShieldMod3230 mecanum_motors;
L298NMotorDriverMega smol_motors(5, 32, 33, 6, 34, 35);

Wheelbase* wheelbase = new Wheelbase(5.0625, 4.386, 2.559);

// Structs and enums
struct Move {
  unsigned short direction;
  unsigned short time;
};

enum States {
  DRIVING = 0,
  WAITING_TO_START = 1,
};

int main() {
  init();  // Initialize board
  Serial.begin(9600);
  Serial2.begin(9600);
  Serial2.setTimeout(10000);

  JsonDocument doc;

  mecanum_motors.init();
  mecanum_motors.enableDrivers();

  smol_motors.init();

  loop(doc);
}

void loop(JsonDocument doc) {
  std::queue<Move>* moveQueue = new std::queue<Move>();
  States state = WAITING_TO_START;

  // LOOP BEGINS
  while (true) {
    switch (state) {
      case WAITING_TO_START:
        debugPrintln("WAITING TO START");
        read_serial(doc);
        if (doc.isNull()) {
          continue;
        } else if (doc.containsKey("d")) {
          debugPrintln("ABOUT TO DESERIALIZE D KEY");
          deserializeDKeyIntoQueue(moveQueue, doc);
          debugPrintln("DONE DESERIALIZING");
          state = DRIVING;
        }
        break;
      case DRIVING:
        debugPrintln("JUST ENTERED DRIVING STATE");
        if (moveQueue->empty()) {
          debugPrintln("MOVEQUEUE EMPTY.");
          state = WAITING_TO_START;  // Go back to waiting state if queue is empty
          debugPrintln("STATE SET TO WAITING TO START!");
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

    debugPrint("Pushed Move - Direction: ");
    debugPrint(currentMove.direction);
    debugPrint(", Time: ");
    debugPrintln(currentMove.time);
  }

  debugPrint("Total Moves in Queue: ");
  debugPrintln(moveQueue->size());
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
  float wheelSpeeds[NUMBER_OF_WHEELS];  // Initialize motor speeds

  Move nextMove = getNextMoveFromQueue(moveQueue);
  int delayTime = nextMove.time;
  int motorMax = 400;
  switch (nextMove.direction) {
    debugPrint("NEXTMOVE.DIRECTION: ");
    debugPrintln(nextMove.direction);

    case 1:  // forwards
      debugPrintln("Driving: Forwards");
      wheelbase->computeWheelSpeeds(0, 10, 0, wheelSpeeds);
      runMotorsWithBlockingDelay(delayTime, wheelSpeeds, motorMax, false);
      break;
    case 2:  // left
      debugPrintln("Driving: Left");
      wheelbase->computeWheelSpeeds(-10, 0, 0, wheelSpeeds);
      runMotorsWithBlockingDelay(delayTime, wheelSpeeds, motorMax, false);
      break;
    case 3:  // backwards
      debugPrintln("Driving: Backwards");
      wheelbase->computeWheelSpeeds(0, -10, 0, wheelSpeeds);
      runMotorsWithBlockingDelay(delayTime, wheelSpeeds, motorMax, false);
      break;
    case 4:  // right
      debugPrintln("Driving: Right");
      wheelbase->computeWheelSpeeds(10, 0, 0, wheelSpeeds);
      runMotorsWithBlockingDelay(delayTime, wheelSpeeds, motorMax, false);
      break;
    case 5:  // rotate ccw
      debugPrintln("Driving: rotate ccw");
      // Rotations are more sensitive. I hand calculated 1.059 to match our map scale
      wheelbase->computeWheelSpeeds(0, 0, 1.059, wheelSpeeds);
      runMotorsWithBlockingDelay(delayTime, wheelSpeeds, motorMax, false);
      break;
    case 6:  // rotate cw
      debugPrintln("Driving: rotate cw");
      // Rotations are more sensitive. I hand calculated 1.059 to match our map scale
      wheelbase->computeWheelSpeeds(0, 0, -1.059, wheelSpeeds);
      runMotorsWithBlockingDelay(delayTime, wheelSpeeds, motorMax, false);
      break;
    case 7:  // lift motor
      debugPrintln("Driving: lift motor");
      runMotorsWithBlockingDelay(delayTime, nullptr, motorMax, true);
      break;
    case 8:  // belt motor
      debugPrintln("Driving: belt motor");
      runMotorsWithBlockingDelay(delayTime, nullptr, motorMax, false);
      break;
    default:
      debugPrint("Unexpected input in direction switch: ");
      debugPrintln(nextMove.direction);
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
    debugPrint("Wheel Speeds before mapping: ");
    for (int i = 0; i < NUMBER_OF_WHEELS; i++) {
      debugPrint(wheelSpeeds[i]);
      debugPrint(" ");  // Space between values for readability
    }

    debugPrintln("");                    // New line after printing all speeds
    mapWheelSpeeds(wheelSpeeds, speed);  // mutates motorSpeeds

    debugPrint("Wheel Speeds after mapping: ");
    for (int i = 0; i < NUMBER_OF_WHEELS; i++) {
      debugPrint(wheelSpeeds[i]);
      debugPrint(" ");  // Space between values for readability
    }
    debugPrintln("");  // New line after printing all speeds

    // Negative signs are to account for polarity of motors. Motors are wired to positive goes to A and negative to B
    mecanum_motors.setSpeeds(wheelSpeeds[0], -wheelSpeeds[1], wheelSpeeds[2], -wheelSpeeds[3]);

  } else {
    if (lift_motor) {
      smol_motors.setM1Speed(speed);
    } else {
      smol_motors.setM2Speed(speed);
    }
  }

  delay(delayTime * 1000);

  // turns off motors after delay
  if (wheelSpeeds) {
    mecanum_motors.setSpeeds(0, 0, 0, 0);
  } else {
    if (lift_motor) {
      smol_motors.setM1Speed(0);
    } else {
      smol_motors.setM2Speed(0);
    }
  }
}

void mapWheelSpeeds(float* wheelSpeeds, unsigned long maxSpeed) {

  for (int i = 0; i < NUMBER_OF_WHEELS; i++) {
    wheelSpeeds[i] = map(wheelSpeeds[i], -3.91, 3.91, -1 * maxSpeed, maxSpeed);
  }
}
