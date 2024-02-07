#include <Arduino.h>
#include <ArduinoSTL.h>
#include <DualTB9051FTGMotorShieldMod3230.h>

using namespace std;

DualTB9051FTGMotorShieldMod3230 md;

int main() {
    init();  // Initialize board
    Serial.begin(9600);
    Serial3.begin(9600);


    md.enableDrivers();
    loop();
}

void loop() {
  while (true) {

    
    md.setSpeeds(180, 250, 500, 254); //sets speeds of all 4 mecanum wheel motors
  }
}
