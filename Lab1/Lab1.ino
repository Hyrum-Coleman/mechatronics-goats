#include <Arduino.h>
#include <SoftwareSerial.h>
#include <ArduinoSTL.h>

using namespace std;

unsigned long time = 0;
unsigned long timeOld = 0;

int main() {
  init(); // Needed to get the board moving since we're not using setup() and loop()
  Serial.begin(9600);


  while (true) {
    time = millis();

    if ( time-timeOld >= 2000 ) {
      cout << "Hello World - " << (float)time /1000 << " sec" << endl;
      timeOld = time;
    }
  }
}