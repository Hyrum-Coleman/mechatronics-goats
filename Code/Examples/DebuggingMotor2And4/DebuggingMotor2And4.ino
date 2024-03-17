#include <L298NMotorDriverMega.h>
#include <DualTB9051FTGMotorShieldMod3230.h>

DualTB9051FTGMotorShieldMod3230 wheels;

String message = "";

void setup() {

  wheels.init();
  wheels.enableDrivers();
  Serial2.begin(9600);
}

void loop() {

  if (Serial2.available() > 2) {
    message = Serial2.readStringUntil('\n');
    Serial2.println(message);
  }

  if (message.equals("Go!")) {

    wheels.setSpeeds(0, 200, 0, 0);
    delay(500);
    wheels.setSpeeds(0, 0, 0, 0);
    delay(500);
    wheels.setSpeeds(0, 0, 0, -200);
    delay(500);
    wheels.setSpeeds(0, 0, 0, 0);
    delay(500);


    message = "";
  }
}
