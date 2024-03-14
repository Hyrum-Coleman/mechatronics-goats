#include <Encoder.h>
#include <DualTB9051FTGMotorShieldMod3230.h>

DualTB9051FTGMotorShieldMod3230 wheels;

// Define encoders for each wheel
Encoder gEnc1(18, 22);  // wheel 1 (interrupt pin, non-interrupt pin)
Encoder gEnc2(3, 24);   // wheel 2 (interrupt pin, non-interrupt pin)
Encoder gEnc3(2, 26);   // wheel 3 (interrupt pin, non-interrupt pin)
Encoder gEnc4(19, 28);  // wheel 4 (interrupt pin, non-interrupt pin)

// Variables to store the previous counts for each encoder
int32_t prevCount1 = 0;
int32_t prevCount2 = 0;
int32_t prevCount3 = 0;
int32_t prevCount4 = 0;

unsigned long prevTime = 0;

void setup() {
  Serial.begin(9600);
  wheels.init();
  wheels.enableDrivers();
  // Initialize previous counts for each encoder
  prevCount1 = gEnc1.read();
  prevCount2 = gEnc2.read();
  prevCount3 = gEnc3.read();
  prevCount4 = gEnc4.read();
  prevTime = millis();
  wheels.setSpeeds(100, -100, 100, -100);
}

void loop() {
  if (millis() - prevTime >= 250) {  // Update and display speeds every quarter second
    displayEncoderSpeeds();
    prevTime = millis();
  }
}

void displayEncoderSpeeds() {
  // Read current counts for each encoder
  int32_t currCount1 = -gEnc1.read();
  int32_t currCount2 = gEnc2.read();
  int32_t currCount3 = -gEnc3.read();
  int32_t currCount4 = gEnc4.read();

  unsigned long currTime = millis();

  // Calculate and display speeds for each wheel
  float speed1 = calculateSpeed(currCount1, prevCount1, currTime, prevTime);
  float speed2 = calculateSpeed(currCount2, prevCount2, currTime, prevTime);
  float speed3 = calculateSpeed(currCount3, prevCount3, currTime, prevTime);
  float speed4 = calculateSpeed(currCount4, prevCount4, currTime, prevTime);

  Serial.print("Counts: ");
  Serial.print(currCount1);
  Serial.print(" | ");
  Serial.print(currCount2);
  Serial.print(" | ");
  Serial.print(currCount3);
  Serial.print(" | ");
  Serial.print(currCount4);
  Serial.print(" | Speeds: ");

  Serial.print(speed1);
  Serial.print(" | ");
  Serial.print(speed2);
  Serial.print(" | ");
  Serial.print(speed3);
  Serial.print(" | ");
  Serial.print(speed4);
  Serial.println("");


  // Update previous counts for each encoder
  prevCount1 = currCount1;
  prevCount2 = currCount2;
  prevCount3 = currCount3;
  prevCount4 = currCount4;
}

float calculateSpeed(long currentCount, long previousCount, unsigned long currentTime, unsigned long previousTime) {
  return (currentCount - previousCount) / ((currentTime - previousTime) / 1000.0);  // Counts per second
}
