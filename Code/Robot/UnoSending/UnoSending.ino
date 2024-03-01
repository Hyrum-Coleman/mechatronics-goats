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