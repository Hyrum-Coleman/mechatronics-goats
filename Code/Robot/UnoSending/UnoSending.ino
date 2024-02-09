#include <SoftwareSerial.h>
#include <ArduinoJson.h>

JsonDocument doc;

SoftwareSerial sendingSerial(2, 3);

void setup() {
  Serial.begin(9600);
  sendingSerial.begin(9600);
    doc["d"] = "We drive";

}

void loop() {
  // put your main code here, to run repeatedly:
  serializeJson(doc, sendingSerial);
  Serial.println("Message sent");
  delay(1000);
}
