#include <L298NMotorDriverMega.h>
#include <DualTB9051FTGMotorShieldMod3230.h>
#include <Adafruit_APDS9960.h>

L298NMotorDriverMega l2_driver(5, 34, 32, 6, 33, 35);
DualTB9051FTGMotorShieldMod3230 wheels;
Adafruit_APDS9960 apds;

String message = "";

void setup() {
  Serial.begin(9600);
  Serial2.begin(9600);

  l2_driver.init();

  wheels.init();
  wheels.enableDrivers();

  apds.enable();

  if (!apds.begin()) {
    Serial.println("Error initializizing APDS9960");
  }

  apds.enableColor(true);
}

void loop() {
  static int r;
  static int g;
  static int b;
  static int c;

  if (Serial2.available() > 2) {
    message = Serial2.readStringUntil('\n');
    Serial2.println(message);
  }

  if (!message.equals("Go!")) {
    return;
  }

  if (!apds.colorDataReady()) {
    return;
  }

  apds.getColorData(r, g, b, c);


  // very rudimentary color detection logic:

  // if red block, turn on wheels
  if (r < 226 && r > 142) {
    Serial.println("Red block detected");
    wheels.setSpeeds(200, 200, 200, 200);
  } else if (b > 75) {  // if blue block turn on belt forwards
    Serial.println("Blue block detected");
    l2_driver.setM2Speed(200);
  } else if (r > 250 && g > 100) {  // if yellow block, drive belt backwards
    Serial.println("Yellow block detected");
    l2_driver.setM2Speed(-200);
  } else {  // not within rudimentary bounds, print value
    Serial.println("Uncertain about the color");
    Serial.print("RGB: (");
    Serial.print(r);
    Serial.print(", ");
    Serial.print(g);
    Serial.print(", ");
    Serial.print(b);
    Serial.print(") Clear Channel: ");
    Serial.println(c);
  }


  delay(500);
  wheels.setSpeeds(0, 0, 0, 0);
  l2_driver.setM2Speed(0);
  message = "";
}
