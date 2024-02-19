#include <Arduino.h>
#include <ArduinoSTL.h>
#include <ArduinoJson.h>
#include <StreamUtils.h>
#include <DualTB9051FTGMotorShieldMod3230.h>
#include <L298NMotorDriverMega.h>
#include <queue>
#include "Wheelbase.h"

using namespace std;

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
          for (JsonObject obj : doc["d"].as<JsonArray>()) {
            Move currentMove;
            currentMove.direction = obj["a"];
            currentMove.time = obj["t"];
            moveQueue->push(currentMove);
          }
          state = DRIVING;
        }
        break;
      case DRIVING:
        driving_logic(moveQueue, state);
        break;
    }
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

void driving_logic(queue<Move>* moveQueue, States& currentState) {
  if (moveQueue->empty()) {
    currentState = WAITING_TO_START;  // Go back to waiting state if queue is empty
  }

  Move nextMove = moveQueue->front();
  moveQueue->pop();

  float motorSpeeds[4] = { 0, 0, 0, 0 };  // Initialize motor speeds

  switch (nextMove.direction) {
    case 1:  // forwards
      wheelbase->computeWheelSpeeds(10, 0, 0, motorSpeeds);
      run_motors_with_blocking_delay(nextMove, motorSpeeds, true, false);
      break;
    case 2:  // left
      wheelbase->computeWheelSpeeds(0, -10, 0, motorSpeeds);
      run_motors_with_blocking_delay(nextMove, motorSpeeds, true, false);
      break;
    case 3:  // backwards
      wheelbase->computeWheelSpeeds(-10, 0, 0, motorSpeeds);
      run_motors_with_blocking_delay(nextMove, motorSpeeds, true, false);
      break;
    case 4:  // right
      wheelbase->computeWheelSpeeds(0, 10, 0, motorSpeeds);
      run_motors_with_blocking_delay(nextMove, motorSpeeds, true, false);
      break;
    case 5:
      run_motors_with_blocking_delay(nextMove, motorSpeeds, false, true);
      break;
    case 6:
      run_motors_with_blocking_delay(nextMove, motorSpeeds, false, false);
      break;
    default:
      Serial.println("Unexpected input in direction switch");
      Serial.println(nextMove.direction);
      break;
  }
}

void run_motors_with_blocking_delay(Move& nextMove, float* motorSpeeds, bool mec_motors, bool lift_motor) {


  if (mec_motors) {
    for (int i = 0; i < 4; i++) {
      motorSpeeds[i] = map(motorSpeeds[i], -3.91, 3.91, -200, 200);
      Serial.println(motorSpeeds[i]);
    }
    mecanum_motors.setSpeeds(motorSpeeds[0], motorSpeeds[1], motorSpeeds[2], motorSpeeds[3]);
  } else {
    if (lift_motor) {
      smol_motors.setM1Speed(200);
    } else {
      smol_motors.setM2Speed(200);
    }
  }

  delay(nextMove.time);

  if (mec_motors) {
    mecanum_motors.setSpeeds(0, 0, 0, 0);
  } else {
    if (lift_motor) {
      smol_motors.setM1Speed(200);
    } else {
      smol_motors.setM2Speed(200);
    }
  }
}
