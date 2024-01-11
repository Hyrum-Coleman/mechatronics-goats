#include <Arduino.h>
#include <SoftwareSerial.h>
#include <ArduinoSTL.h>
#include "TimerUtility.h"

using namespace std;

/*
void timerCallback() {
  cout << "Hello World - " << (float)millis() / 1000 << " sec" << endl;
}
*/

int main() {
    init();  // Initialize board
    Serial.begin(9600);

    /*
    // Specify the type of the callback directly
    Timer<decltype(timerCallback)*> timer(500, timerCallback);
    */

    
    // Lambda function for the timer callback
    auto lambdaCallback = []() {
        cout << "Hello World - " << (float)millis() / 1000 << " sec" << endl;
    };

    // Specify the type of the callback as the type of the lambda
    Timer<decltype(lambdaCallback)> timer(500, lambdaCallback);
    

    while (true) {
        timer.check();
    }
}
