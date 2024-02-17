#include <Arduino.h>
#include <ArduinoSTL.h>
#include <ArduinoJson.h>
#include <StreamUtils.h>
#include <DualTB9051FTGMotorShieldMod3230.h>
#include <L298NMotorDriverMega.h>
#include <queue>
#include <Wheelbase.h>

using namespace std;

DualTB9051FTGMotorShieldMod3230 mecanum_motors;
L298NMotorDriverMega smol_motors(5, 32, 33, 6, 34, 35);

Wheelbase* wheelbase = new Wheelbase(5.0625, 4.386, 2.559);

struct Move {
  unsigned short direction;
  unsigned short time;
};


enum states {
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
  queue<Move>* sillyQueue = new queue<Move>();
  states state = WAITING_TO_START;  // Holds incoming data from Serial2
  while (true) {

    switch (state) {
      case WAITING_TO_START:
        read_serial(doc);  // mutates doc
        if (doc.isNull()) {
          continue;
        } else if (doc.containsKey("d")) {
          Move sillyGuy;

          for (JsonObject obj : doc["d"].as<JsonArray>()) {
            sillyGuy.direction = obj["a"];
            sillyGuy.time = obj["t"];
          }
          // sillyGuy.direction = doc["d"][0]["a"];
          // sillyGuy.time = doc["d"][0]["t"];
          Serial.println(sillyGuy.direction);
          Serial.println(sillyGuy.time);

          sillyQueue->push(sillyGuy);
          state = DRIVING;
        }
        break;
      case DRIVING:
        float motorSpeeds[4];

        Move nextMove = sillyQueue->back();
        sillyQueue->pop();

        switch (nextMove.direction) {
          case 1:  // forwards
            wheelbase->computeWheelSpeeds(10, 0, 0, motorSpeeds);
            break;
          case 2:  // left
            wheelbase->computeWheelSpeeds(0, -10, 0, motorSpeeds);
            break;
          case 3:  // backwards
            wheelbase->computeWheelSpeeds(-10, 0, 0, motorSpeeds);
            break;
          case 4:  // right
            wheelbase->computeWheelSpeeds(0, 10, 0, motorSpeeds);
            break;
          case 5:
            smol_motors.setM1Speed(200);
            break;
            case 6:
            smol_motors.setM2Speed(200);
          default:
            Serial.println("unexpected input in direction switch (line 86)");
            Serial.println(nextMove.direction);
            break;
        }
        for (int i = 0; i < 4; i++) {
          motorSpeeds[i] = map(motorSpeeds[i], -3.91, 3.91, -50, 50);
          Serial.println(motorSpeeds[i]);
        }
        mecanum_motors.setSpeeds(motorSpeeds[0], motorSpeeds[1], motorSpeeds[2], motorSpeeds[3]);  //sets speeds of all 4 mecanum wheel motors
        delay(nextMove.time * 1000);
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
