#include <Arduino.h>
#define gamer_pin 13

int main() {
  init();
  pinMode(gamer_pin, OUTPUT);
  digitalWrite(gamer_pin, HIGH);

  while (true) {
    
    delay(100);
    digitalWrite(gamer_pin, !digitalRead(gamer_pin));
  }
}
