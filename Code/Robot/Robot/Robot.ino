#include <Arduino.h>
#include <ArduinoSTL.h>
#include <DualTB9051FTGMotorShieldMod3230.h>

using namespace std;

DualTB9051FTGMotorShieldMod3230 md;

enum states {
  DRIVING = 0,
};

int main() {
  init();  // Initialize board
  Serial.begin(9600);
  Serial3.begin(9600);


  md.enableDrivers();
  loop();
}

void loop() {
  bool start;
  while (true) {
    // Check if data is available to read from Serial3
    if (Serial3.available() > 0) {  // zero will change here so I'm not going to try and predict what it will be
      // Read the incoming byte from Serial3
      char incomingByte = Serial3.read();

      // Check if the incoming byte is '1' or '0' and set the start flag accordingly
      if (incomingByte == '1') {
        start = true;
      } else if (incomingByte == '0') {
        start = false;
      }
    }

    if (start) {
      md.setSpeeds(180, 250, 500, 254);  //sets speeds of all 4 mecanum wheel motors
    }
  }
}
