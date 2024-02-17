#include <SoftwareSerial.h>
#include <StreamUtils.h>
#include <ArduinoJson.h>

JsonDocument doc;

SoftwareSerial sendingSerial(2, 3);

void setup() {
  Serial.begin(9600);
  sendingSerial.begin(9600);

  JsonArray arr = doc.createNestedArray("d");
  JsonObject obj = arr.createNestedObject();
  obj["a"] = 2;
  obj["t"] = 2;
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


  WriteLoggingStream loggingStream(sendingSerial, Serial);
  serializeJson(doc, loggingStream);
  sendingSerial.println();
  delay(1000);
}
