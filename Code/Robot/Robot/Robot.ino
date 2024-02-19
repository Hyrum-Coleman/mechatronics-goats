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
  queue<Move>* moveQueue = new queue<Move>();
  States state = WAITING_TO_START;
  while (true) {
    switch (state) {
      case WAITING_TO_START:
        read_serial(doc);
        if (doc.isNull()) {
          continue;
        } else if (doc.containsKey("d")) {
          deserializeDKeyIntoQueue(moveQueue, doc);
          state = DRIVING;
        }
        break;
      case DRIVING:
        if (moveQueue->empty()) {
          state = WAITING_TO_START;  // Go back to waiting state if queue is empty
        }
        driving_logic(moveQueue);
        break;
    }
  }
}

void deserializeDKeyIntoQueue(queue<Move>* moveQueue, JsonDocument& doc) {
  for (JsonObject obj : doc["d"].as<JsonArray>()) {
    Move currentMove;
    currentMove.direction = obj["a"];
    currentMove.time = obj["t"];
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

void driving_logic(queue<Move>* moveQueue) {
  float motorSpeeds[NUMBER_OF_WHEELS];  // Initialize motor speeds

  Move nextMove = getNextMoveFromQueue(moveQueue);
  int time = nextMove.time;
  switch (nextMove.direction) {
    case 1:  // forwards
      wheelbase->computeWheelSpeeds(10, 0, 0, motorSpeeds);
      run_motors_with_blocking_delay(time, motorSpeeds, 200, false);
      break;
    case 2:  // left
      wheelbase->computeWheelSpeeds(0, -10, 0, motorSpeeds);
      run_motors_with_blocking_delay(time, motorSpeeds, 200, false);
      break;
    case 3:  // backwards
      wheelbase->computeWheelSpeeds(-10, 0, 0, motorSpeeds);
      run_motors_with_blocking_delay(time, motorSpeeds, 200, false);
      break;
    case 4:  // right
      wheelbase->computeWheelSpeeds(0, 10, 0, motorSpeeds);
      run_motors_with_blocking_delay(time, motorSpeeds, 200, false);
      break;
    case 5:  // lift motor
      run_motors_with_blocking_delay(time, nullptr, 200, true);
      break;
    case 6:  // belt motor
      run_motors_with_blocking_delay(time, nullptr, 200, false);
      break;
    default:
      Serial.println("Unexpected input in direction switch");
      Serial.println(nextMove.direction);
      break;
  }
}

Move getNextMoveFromQueue(queue<Move>* queueToPopFrom) {
  Move retMove = queueToPopFrom->back();
  queueToPopFrom->pop();
  return retMove;
}

void run_motors_with_blocking_delay(int delayTime, float* motorSpeeds, unsigned long speed, bool lift_motor) {

  // if motorSpeeds is accessesed outside this if, a segfault will be issued :trollface:
  if (motorSpeeds) {
    map_motor_speeds(motorSpeeds, speed);  // mutates motorSpeeds
    mecanum_motors.setSpeeds(motorSpeeds[0], motorSpeeds[1], motorSpeeds[2], motorSpeeds[3]);
  } else {
    if (lift_motor) {
      smol_motors.setM1Speed(speed);
    } else {
      smol_motors.setM2Speed(speed);
    }
  }

  delay(delayTime * 1000);

  // turns off motors after delay
  if (motorSpeeds) {
    mecanum_motors.setSpeeds(0, 0, 0, 0);
  } else {
    if (lift_motor) {
      smol_motors.setM1Speed(0);
    } else {
      smol_motors.setM2Speed(0);
    }
  }
}

void map_motor_speeds(float* motorSpeeds, unsigned long maxSpeed) {
  for (int i = 0; i < NUMBER_OF_WHEELS; i++) {
    motorSpeeds[i] = map(motorSpeeds[i], -3.91, 3.91, -1 * maxSpeed, maxSpeed);
    Serial.println(motorSpeeds[i]);
  }
}
