#include <Arduino.h>
#include <SoftwareSerial.h>
#include <ArduinoSTL.h>
#include "TimerUtility.h"

using namespace std; //lol

void timerCallback() {
  cout << "Hello World - " << (float)millis() / 1000 << " sec" << endl;
}

int main() {
  init();  // Needed to get the board moving since we're not using setup() and loop()
  Serial.begin(9600);

  Timer timer(2000, timerCallback);

  while (true) {
    timer.check();
  }
}