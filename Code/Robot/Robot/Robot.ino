#include <Arduino.h>
#include <ArduinoSTL.h>
#include <ArduinoJson.h>
#include <DualTB9051FTGMotorShieldMod3230.h>

using namespace std;

DualTB9051FTGMotorShieldMod3230 md;

enum states {
  DRIVING = 0,
  WAITING_TO_START = 1,
};

int main() {
  init();  // Initialize board
  Serial.begin(9600);
  Serial3.begin(9600);

  JsonDocument doc;


  md.enableDrivers();
  loop(doc);
}

void loop(JsonDocument doc) {
  states state = WAITING_TO_START;    // Holds incoming data from Serial3
  while (true) {

    switch (state) {
      case WAITING_TO_START:
        read_serial(doc); // mutates doc
        if (doc.isNull()) {
          continue;
        } else if (doc.containsKey("d")) {
          state = DRIVING;
        }
        break;
      case DRIVING:
        md.setSpeeds(150, 250, 400, 300);  //sets speeds of all 4 mecanum wheel motors
        break;
    }
  }
}

void read_serial(JsonDocument doc) {
  DeserializationError error = deserializeJson(doc, Serial3);

    // Test if parsing succeeds.
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }
}
