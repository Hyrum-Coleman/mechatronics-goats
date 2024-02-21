#include <SoftwareSerial.h>
#include <StreamUtils.h>
#include <ArduinoJson.h>

JsonDocument doc;

SoftwareSerial sendingSerial(2, 3);

void setup() {
  Serial.begin(9600);
  sendingSerial.begin(9600);

  JsonArray arr = doc.createNestedArray("d");
  JsonObject obj1 = arr.createNestedObject();
  obj1["a"] = 1;
  obj1["t"] = 1000;
  JsonObject obj2 = arr.createNestedObject();
  obj2["a"] = 2;
  obj2["t"] = 1000;
  JsonObject obj3 = arr.createNestedObject();
  obj3["a"] = 3;
  obj3["t"] = 1000;
  JsonObject obj4 = arr.createNestedObject();
  obj4["a"] = 4;
  obj4["t"] = 1000;
  JsonObject obj5 = arr.createNestedObject();
  obj5["a"] = 5;
  obj5["t"] = 1000;
  JsonObject obj6 = arr.createNestedObject();
  obj6["a"] = 6;
  obj6["t"] = 1000;
  JsonObject obj7 = arr.createNestedObject();
  obj7["a"] = 7;
  obj7["t"] = 500;
  JsonObject obj8 = arr.createNestedObject();
  obj8["a"] = 8;
  obj8["t"] = 500;

  serializeJson(doc, sendingSerial);
}

void loop() {
  // Sending
  if (Serial.available() > 4) {
    deserialize_from_serial_input();
  }

  // Receiving
  if (sendingSerial.available()) {
    Serial.println(sendingSerial.readStringUntil('\n'));
  }
}

void deserialize_from_serial_input() {
  DeserializationError error = deserializeJson(doc, Serial);

  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    doc.clear();
    return;
  }

  WriteLoggingStream loggingStream(sendingSerial, Serial);
  serializeJson(doc, loggingStream);
  sendingSerial.println();
  return;
}