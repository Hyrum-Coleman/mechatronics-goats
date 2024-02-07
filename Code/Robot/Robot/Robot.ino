#include <Arduino.h>
#include <ArduinoSTL.h>
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


  md.enableDrivers();
  loop();
}

void loop() {
  states state = WAITING_TO_START;
  String inputString = "";            // Holds incoming data from Serial3
  boolean messageInProgress = false;  // Indicates if message parsing is in progress
  while (true) {

    switch (state) {
      case WAITING_TO_START:
        String message = read_serial(inputString, messageInProgress);
        if (message == "") {
          continue;
        } else if (message == "<1>") {
          state = DRIVING;
        }
        break;
      case DRIVING:
        md.setSpeeds(150, 250, 400, 300);  //sets speeds of all 4 mecanum wheel motors
        break;
    }
  }
}

String read_serial(String inputString, bool messageInProgress) {
  // Check if data is available to read from Serial3
  while (Serial3.available() > 0) {  // read every byte!
    // Read the incoming byte from Serial3
    char inChar = Serial3.read();

    // First character qualifier <
    if (inChar == '<' && !messageInProgress) {
      inputString = "";          // Clear the string
      messageInProgress = true;  // Start collecting message
      continue;                  // Skip further processing for this character
    }
    // Last character qualifier >
    if (inChar == '>' && messageInProgress) {
      // Process complete message
      messageInProgress = false;  // Reset for next message
      continue;                   // Skip further processing for this character
    }

    if (messageInProgress) {
      inputString += inChar;
    }
  }

  return inputString;
}
