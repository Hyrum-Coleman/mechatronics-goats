void setup() {
  // Open serial communications with computer and wait for port to open:
  Serial.begin(9600);

  // Print a message to the computer through the USB
  Serial.println("Hello Computer!");

  // Open serial communications with the other Arduino board
  Serial3.begin(9600);

  // Send a message to the other Arduino board
  Serial3.print("Hello other Arduino!");
}

void loop() { // run over and over

  if (Serial.available()) {
    Serial3.println(Serial.readStringUntil('\n'));
  }

  if (Serial3.available()) {
    Serial.println(Serial3.readStringUntil('\n'));
  }
}
