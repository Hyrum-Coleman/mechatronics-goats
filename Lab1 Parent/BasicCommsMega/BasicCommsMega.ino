void setup() {
  // Open serial communications with computer and wait for port to open:
  Serial.begin(9600);

  // Print a message to the computer through the USB
  Serial.println("Hello Computer!");

  // Open serial communications with the other Arduino board
  Serial1.begin(9600);

  // Send a message to the other Arduino board
  Serial1.print("Hello other Arduino!");
}

void loop() { // run over and over

  if (Serial.available()) {
    Serial1.println(Serial.readStringUntil('\n'));
  }

  if (Serial1.available()) {
    Serial.println(Serial1.readStringUntil('\n'));
  }
}
