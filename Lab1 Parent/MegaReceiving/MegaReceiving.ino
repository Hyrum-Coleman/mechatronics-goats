const int ledPin = 13;
bool receivedLedValue;
int receivedServoAngle;

void setup()  
{
  pinMode(ledPin, OUTPUT);

  // Open serial communications with computer and wait for port to open:
  Serial.begin(9600);

  // Print a message to the computer through the USB
  
  Serial.println("Hello Computer!");

  // Open serial communications with the other Arduino board
  Serial1.begin(9600);

  // Send a message to the other Arduino board
  Serial1.print("Hello other Arduino!");

}

void loop() // run over and over
{
  if (Serial1.available() > 2) {
    if (Serial1.read() == 255) {
      receivedLedValue = Serial1.read();
      receivedServoAngle = Serial1.read();
    }
  }
  digitalWrite(ledPin, receivedLedValue);
  Serial.println(receivedServoAngle);
  //Serial.println(receivedLedValue);

  /*
  // Sending
  if (Serial.available()) {
    Serial1.println(Serial.readStringUntil('\n'));
  }

  // Receiving
  if (Serial1.available()) {
    Serial.println(Serial1.readStringUntil('\n'));
  }
  */
}
