#include <L298NMotorDriverMega.h>
#include <DualTB9051FTGMotorShieldMod3230.h>

L298NMotorDriverMega l2_driver(5, 34, 32, 6, 33, 35);
DualTB9051FTGMotorShieldMod3230 wheels;

String message = "";

void setup() {

  l2_driver.init();


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

    

    message = "";
  }
}
