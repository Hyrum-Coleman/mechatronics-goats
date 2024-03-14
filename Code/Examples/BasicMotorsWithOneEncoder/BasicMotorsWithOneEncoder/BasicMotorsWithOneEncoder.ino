#include <DualTB9051FTGMotorShieldMod3230.h>
#include <Encoder.h>

DualTB9051FTGMotorShieldMod3230 wheels;

String message = "";
Encoder gEnc1(18, 22); // wheel 1 (int, non int)

long prevCount1 = 0;
unsigned long prevTime = 0;

void setup() {
  wheels.init();
  wheels.enableDrivers();
  Serial2.begin(9600);
  prevCount1 = gEnc1.read();
  prevTime = millis();
  Serial2.println("Starting single encoder test script");
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
    displayEncoderSpeed();
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
      case 0: wheels.setSpeeds(255, 0, 0, 0); break;
      case 1: wheels.setSpeeds(0, 0, 0, 0); break; 
      case 2: wheels.setSpeeds(-255, 0, 0, 0); break;
      case 3: 
        wheels.setSpeeds(0, 0, 0, 0);
        step = -1; // reset step for next "Go!" command
        break;
    }

    lastActionTime = currentTime; // update the last action time
    step++; // move to the next step
  }
}

void displayEncoderSpeed() {
  long currCount1 = gEnc1.read();
  unsigned long currTime = millis();

  float speed1 = calculateSpeed(currCount1, prevCount1, currTime, prevTime);

  Serial2.print("Wheel 1 Speed: ");
  Serial2.print(speed1);
  Serial2.println(" counts/s");

  prevCount1 = currCount1;
  prevTime = currTime;
}

float calculateSpeed(long currentCount, long previousCount, unsigned long currentTime, unsigned long previousTime) {
  return (currentCount - previousCount) / ((currentTime - previousTime) / 1000.0); // cps
}
