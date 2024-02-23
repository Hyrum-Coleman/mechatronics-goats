#include <SoftwareSerial.h>
#include <StreamUtils.h>
#include <ArduinoJson.h>

JsonDocument doc;

SoftwareSerial sendingSerial(2, 3);

void setup() {
  Serial.begin(9600);
  sendingSerial.begin(9600);

  //sendTestSequence();
  //sendLineFollowingTest();
  // OR, send json packets in serial monitor
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

void sendTestSequence() {
  DynamicJsonDocument doc(1024);
  JsonArray arr = doc.createNestedArray("d");

  // Drive forwards, left, back, right, rotate CCW then CW
  const int directionValues[] = {1, 2, 3, 4, 5, 6};
  const unsigned long durations[] = {1000, 1000, 1000, 1000, 1000, 1000};

  for (int i = 0; i < 6; i++) {
    JsonObject obj = arr.createNestedObject();
    obj["type"] = "freedrive";
    obj["direction"] = directionValues[i];
    obj["duration"] = durations[i];
  }

  // Move the lift up and down
  JsonObject obj7 = arr.createNestedObject();
  obj7["type"] = "scissor";
  obj7["direction"] = 1;

  JsonObject obj8 = arr.createNestedObject();
  obj8["type"] = "scissor";
  obj8["direction"] = 0;

  // Move the belt forward and back
  JsonObject obj9 = arr.createNestedObject();
  obj9["type"] = "belt";
  obj9["direction"] = 1;
  obj9["duration"] = 2000;

  JsonObject obj10 = arr.createNestedObject();
  obj10["type"] = "belt";
  obj10["direction"] = 0;
  obj10["duration"] = 2000;

  serializeJson(doc, sendingSerial);
}

void sendLineFollowingTest() {
  DynamicJsonDocument doc2(1024);
  JsonArray arr2 = doc2.createNestedArray("d");

  JsonObject obj1 = arr2.createNestedObject();
  obj1["type"] = "linefollow";
  obj1["stopDistance"] = 10;

  serializeJson(doc2, sendingSerial);
}
