#include <SoftwareSerial.h>
#include <StreamUtils.h>
#include <ArduinoJson.h>

JsonDocument doc;

SoftwareSerial sendingSerial(2, 3);

void setup() {
  Serial.begin(9600);
  sendingSerial.begin(9600);
  doc["d"] = 1;
  doc.shrinkToFit();  // optional
  
}

void loop() {


  if (Serial.available() > 4) {
    DeserializationError error = deserializeJson(doc, Serial);

    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
      doc.clear();
    }
  }


  // WriteLoggingStream loggingStream(sendingSerial, Serial);
  serializeJson(doc, sendingSerial);
  sendingSerial.println();
}
