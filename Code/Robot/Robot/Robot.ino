#include <Arduino.h>
#include <ArduinoSTL.h>
#include <ArduinoJson.h>
#include <StreamUtils.h>
#include <DualTB9051FTGMotorShieldMod3230.h>
#include <L298NMotorDriverMega.h>
#include <queue>
#include "Wheelbase.h"

const int NUMBER_OF_WHEELS = 4;

DualTB9051FTGMotorShieldMod3230 mecanum_motors;
L298NMotorDriverMega smol_motors(5, 32, 33, 6, 34, 35);

Wheelbase* wheelbase = new Wheelbase(5.0625, 4.386, 2.559);

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
        Serial2.println("WAITING TO START");
        read_serial(doc);
        if (doc.isNull()) {
          continue;
        } else if (doc.containsKey("d")) {
          Serial2.println("ABOUT TO DESERIALIZE D KEY");
          deserializeDKeyIntoQueue(moveQueue, doc);
          Serial2.println("DONE DESERIALIZING");
          state = DRIVING;
        }
        break;
      case DRIVING:
      Serial2.println("JUST ENTERED DRIVING STATE");
        if (moveQueue->empty()) {
          Serial2.println("MOVEQUEUE EMPTY.");
          state = WAITING_TO_START;  // Go back to waiting state if queue is empty
          Serial2.println("STATE SET TO WAITING TO START!");
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

    Serial2.print("Pushed Move - Direction: ");
    Serial2.print(currentMove.direction);
    Serial2.print(", Time: ");
    Serial2.println(currentMove.time);
  }

  Serial2.print("Total Moves in Queue: ");
  Serial2.println(moveQueue->size());
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
  int motorMax = 200;
  switch (nextMove.direction) {
    Serial2.print("NEXTMOVE.DIRECTION: ");
    Serial2.println(nextMove.direction);
    
    case 1:  // forwards
      Serial2.println("Driving: Forwards");
      wheelbase->computeWheelSpeeds(10, 0, 0, wheelSpeeds);
      runMotorsWithBlockingDelay(delayTime, wheelSpeeds, motorMax, false);
      break;
    case 2:  // left
      Serial2.println("Driving: Left");
      wheelbase->computeWheelSpeeds(0, -10, 0, wheelSpeeds);
      runMotorsWithBlockingDelay(delayTime, wheelSpeeds, motorMax, false);
      break;
    case 3:  // backwards
      Serial2.println("Driving: Backwards");
      wheelbase->computeWheelSpeeds(-10, 0, 0, wheelSpeeds);
      runMotorsWithBlockingDelay(delayTime, wheelSpeeds, motorMax, false);
      break;
    case 4:  // right
      Serial2.println("Driving: Right");
      wheelbase->computeWheelSpeeds(0, 10, 0, wheelSpeeds);
      runMotorsWithBlockingDelay(delayTime, wheelSpeeds, motorMax, false);
      break;
    case 5:  // lift motor
      Serial2.println("Driving: lift motor");
      runMotorsWithBlockingDelay(delayTime, nullptr, motorMax, true);
      break;
    case 6:  // belt motor
      Serial2.println("Driving: belt motor");
      runMotorsWithBlockingDelay(delayTime, nullptr, motorMax, false);
      break;
    default:
      Serial.println("Unexpected input in direction switch");
      Serial.println(nextMove.direction);
      break;
  }
}

Move getNextMoveFromQueue(std::queue<Move>* queueToPopFrom) {
  Move retMove = queueToPopFrom->front();
  //Move retMove = queueToPopFrom->back();

  Serial2.print("Queue size before popping: ");
  Serial2.println(queueToPopFrom->size());
  queueToPopFrom->pop();
  Serial2.print("Queue size after popping: ");
  Serial2.println(queueToPopFrom->size());
  return retMove;
}

void runMotorsWithBlockingDelay(int delayTime, float* wheelSpeeds, unsigned long speed, bool lift_motor) {

  // if motorSpeeds is accessesed outside this if, a segfault will be issued :trollface:
  if (wheelSpeeds) {
    mapWheelSpeeds(wheelSpeeds, speed);  // mutates motorSpeeds
    mecanum_motors.setSpeeds(wheelSpeeds[0], wheelSpeeds[1], wheelSpeeds[2], wheelSpeeds[3]);
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
    Serial.println(wheelSpeeds[i]);
  }
}
