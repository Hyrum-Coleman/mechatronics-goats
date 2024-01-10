#include <SoftwareSerial.h>

SoftwareSerial mySerial(2, 3); // RX, TX
unsigned long t_ms;
float t, t_old;
bool LEDval;
int servoAngle;

void setup()  
{
  // Open serial communications with computer and wait for port to open:
  Serial.begin(9600);

  // Print a message to the computer through the USB
  Serial.println("Hello Computer!");

  // Open serial communications with the other Arduino board
  mySerial.begin(9600);

  // Send a message to the other Arduino board
  mySerial.print("Hello other Arduino!");
 
}

void loop() // run over and over
{
  t_ms = millis();
  t = float(t_ms) / 1000.0;

  if (t - t_old > 1) {
    if (LEDval == 0){
      LEDval = 1;
    } else{
      LEDval = 0;
    }
    t_old = t;
  }
  Serial.println(LEDval);

  servoAngle = int(125 * sin(t)) + 127;

  mySerial.write(255);         // Send starting flag value
  mySerial.write(LEDval);      // Send LED command
  mySerial.write(servoAngle);  // Send servo command
  delay(20);


  // // Sending
  // if (Serial.available()) {
  //   mySerial.println(Serial.readStringUntil('\n'));
  // }

  // // Receiving
  // if (mySerial.available()) {
  //   Serial.println(mySerial.readStringUntil('\n'));
  // }


}
