#include <DualTB9051FTGMotorShieldMod3230.h>
#include <Encoder.h>

DualTB9051FTGMotorShieldMod3230 wheels;

String message = "";
Encoder gEnc1(18, 22); // wheel 1 (int, non int)
Encoder gEnc2(3, 24);  // wheel 2 (int, non int)
Encoder gEnc3(2, 26);  // wheel 3 (int, non int)
Encoder gEnc4(19, 28); // wheel 4 (int, non int)

long prevCount1 = 0, prevCount2 = 0, prevCount3 = 0, prevCount4 = 0;
unsigned long prevTime = 0;

void setup() {
  wheels.init();
  wheels.enableDrivers();
  Serial2.begin(9600);
  prevCount1 = gEnc1.read();
  prevCount2 = gEnc2.read();
  prevCount3 = gEnc3.read();
  prevCount4 = gEnc4.read();
  prevTime = millis();
}

void loop() {
  if (Serial2.available() > 2) {
    message = Serial2.readStringUntil('\n');
    Serial2.println(message);
  }

  if (message.equals("Go!")) {
    runTestSequence();
    message = "";
  }

  if (millis() - prevTime >= 250) { // print speeds every quarter sec
    displayEncoderSpeeds();
    prevTime = millis();
  }
}

void runTestSequence() {
  static unsigned long lastActionTime = 0;
  const unsigned long actionInterval = 500; // time between actions
  static int step = 0;

  unsigned long currentTime = millis();
  if (currentTime - lastActionTime >= actionInterval) {
    switch (step) {
      case 0: wheels.setSpeeds(255, -255, 255, -255); break;
      case 1: wheels.setSpeeds(0, 0, 0, 0); break;
      case 2: wheels.setSpeeds(-255, 255, -255, 255); break;
      case 3: wheels.setSpeeds(0, 0, 0, 0); break;
      case 4: wheels.setSpeeds(255, 255, -255, -255); break;
      case 5: wheels.setSpeeds(0, 0, 0, 0); break;
      case 6: wheels.setSpeeds(-255, -255, 255, 255); break;
      case 7: wheels.setSpeeds(0, 0, 0, 0); break;
      case 8: wheels.setSpeeds(255, 255, 255, 255); break;
      case 9: wheels.setSpeeds(0, 0, 0, 0); break;
      case 10: wheels.setSpeeds(-255, -255, -255, -255); break;
      case 11: 
        wheels.setSpeeds(0, 0, 0, 0);
        step = -1; // reset step for next "Go!" command
        break;
    }

    lastActionTime = currentTime; // update the last action time
    step++; // move to the next step
  }
}

void displayEncoderSpeeds() {
  long currCount1 = gEnc1.read();
  long currCount2 = gEnc2.read();
  long currCount3 = gEnc3.read();
  long currCount4 = gEnc4.read();
  unsigned long currTime = millis();

  float speed1 = calculateSpeed(currCount1, prevCount1, currTime, prevTime);
  float speed2 = calculateSpeed(currCount2, prevCount2, currTime, prevTime);
  float speed3 = calculateSpeed(currCount3, prevCount3, currTime, prevTime);
  float speed4 = calculateSpeed(currCount4, prevCount4, currTime, prevTime);

  Serial2.print("Wheel 1 Speed: ");
  Serial2.print(speed1);
  Serial2.println(" counts/s");
  Serial2.print("Wheel 2 Speed: ");
  Serial2.print(speed2);
  Serial2.println(" counts/s");
  Serial2.print("Wheel 3 Speed: ");
  Serial2.print(speed3);
  Serial2.println(" counts/s");
  Serial2.print("Wheel 4 Speed: ");
  Serial2.print(speed4);
  Serial2.println(" counts/s");

  // Update previous counts and time for the next calculation
  prevCount1 = currCount1;
  prevCount2 = currCount2;
  prevCount3 = currCount3;
  prevCount4 = currCount4;
  prevTime = currTime;
}

float calculateSpeed(long currentCount, long previousCount, unsigned long currentTime, unsigned long previousTime) {
  return (currentCount - previousCount) / ((currentTime - previousTime) / 1000.0); // cps
}
