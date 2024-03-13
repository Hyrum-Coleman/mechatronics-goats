/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
  * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */ 

//-----------Macro for debug printing----------
// Enable or disable debug prints
#define DEBUG_PRINTS_ENABLED true

#if DEBUG_PRINTS_ENABLED
#define DEBUG_PRINT(x) (Serial2.print(x))
#define DEBUG_PRINTLN(x) (Serial2.println(x))
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif
//---------------------------------------------

//-------------Include dependencies------------
#include <Arduino.h>
#include <ArduinoSTL.h>
#include <math.h>
#include <queue>
#include <stack>
#include <ArduinoJson.h>
#include <StreamUtils.h>
#include <DualTB9051FTGMotorShieldMod3230.h>
#include <L298NMotorDriverMega.h>
#include <QTRSensors.h>
#include <IRremote.hpp>
#include <Adafruit_APDS9960.h>
#include <PID.h>
#include "Wheelbase.h"
#include "types.h"
//---------------------------------------------

//---------------Global Variables--------------
// Quantities
const int cNumberOfWheels = 4;
const uint8_t cSensorCount = 8;
const int cMaxBlocks = 5;
// Parameters
const int cFilterWindowSize = 20;
int gDriveSpeed = 200;
int gRemoteControlDuration = 1000;
unsigned long gLastRCCommandTime = 0;
const unsigned long cRCCommandTimeout = 110;
const unsigned long cReloadTimeout = 5000;
const unsigned int cProximityThreshold = 10;
const float cHallReloadingThreshold = 545;  // This needs to be tested by hand.
// Pins
const int cDistPin1 = A4;  // Left IR rangefinder sensor
const int cDistPin2 = A5;  // Right IR rangefinder sensor
const int cTopLimitSwitchPin = 53;
const int cBottomLimitSwitchPin = 52;
const int cIrRecievePin = 11;
const int cHallSensorPin = A3;
// Sensors
uint16_t gLineSensorValues[cSensorCount];
std::queue<float> gDistSensor1Readings;
std::queue<float> gDistSensor2Readings;
QTRSensors gQtr;
Adafruit_APDS9960 gApds;
float averageRedReadings[3] = { -1, -1, -1 };  // Index 0 for red, 1 for green, 2 for blue
float averageYellowReadings[3] = { -1, -1, -1 };
float averageBlueReadings[3] = { -1, -1, -1 };
// Motors
DualTB9051FTGMotorShieldMod3230 gMecanumMotors;
L298NMotorDriverMega gL2Motors(5, 34, 32, 6, 33, 35);
Wheelbase* gWheelbase = new Wheelbase(5.0625, 4.386, 2.559);
// For keeping track of previous standby state so we can return to it
States gLastStandbyState;

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
  * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */ 

//-----------------ENTRY POINT-----------------
int main() {
  init();  // Initialize board itself
  Serial.begin(9600); // USB Serial Comms
  Serial2.begin(9600); // Radio Serial Comms
  Serial2.setTimeout(10000);

  JsonDocument doc;

  // Initialize both DualTB drivers
  gMecanumMotors.init();
  gMecanumMotors.enableDrivers();

  // Initialize L298N
  gL2Motors.init();

  // Initialize IR array
  gQtr.setTypeRC();
  gQtr.setSensorPins((const uint8_t[]){
                       36, 38, 40, 42, 43, 41, 39, 37 },
                     cSensorCount);

  // Start the IR Reciever
  IrReceiver.begin(cIrRecievePin, true);  // true for enable IR feedback

  if (!gApds.begin()) { // if the color sensor didnt start up correctly
    DEBUG_PRINTLN("Initialization Failed :(");
  } else { // if the color sensor DID start up correctly
    //enable color sensing mode
    gApds.enableColor(true);
    gApds.enableProximity(true);

    /*
            | color_gain    | Gain Multiplier | Note             |
            |---------------|-----------------|------------------|
            | 0x0           | 1x              | Power-on Default |
            | 0x01          | 4x              | Driver Default   |
            | 0x02          | 16x             |                  |
            | 0x03          | 64x             |                  |
            */

    gApds.setADCGain(APDS9960_AGAIN_4X);  // This gain got the best results

    /*
            | prop  | time     | counts| note            |
            |-------|----------|-------|-----------------|
            | 1     | 2.78 ms  | 1025  | Power-on Default|
            | 10    | 27.8 ms  | 10241 |                 |
            | 37    | 103 ms   | 37889 |                 |
            | 72    | 200 ms   | 65535 |                 |
            | 256   | 712 ms   | 65535 | Driver Default  |
            */

    gApds.setADCIntegrationTime(103);  // This integration time (like shutter speed on a camera) got the best results
  }

  setPinModes(); // This is our function to avoid writing pinMode a brazillion times in setup

  loop(doc); // Since we are using int main(), we need to enter the loop manually
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
  * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */ 

void loop(JsonDocument& doc) {
  // For managing sequences of actions
  std::queue<Move>* moveQueue = new std::queue<Move>();
  // For representing the blocks stored on our robots belt. FILO.
  std::stack<Block>* blocks = new std::stack<Block>();
  // Our robots state starts in standbyIR mode since it is the most multi-purpose.
  States state = eStandbyIR;

  // --------------------LOOP BEGINS---------------------
  while (true) {
    switch (state) {
      case eStandbyJSON:
        standbyJSON(doc, moveQueue, state);
        gLastStandbyState = eStandbyJSON;
        break;
      case eStandbyIR:
        standbyIR(doc, moveQueue, blocks, state);
        gLastStandbyState = eStandbyIR;
        break;
      case eMoving:
        executeMoveSequence(moveQueue);
        state = gLastStandbyState;  // return to the state we came from when done moving
        break;
      case eReloading:
        executeReload(blocks);
        state = gLastStandbyState;  // return to the state we came from when done reloading
        break;
      case eSensorDumpMode:
        standbySensorDump(state);
        break;
      case eStandbyRC:
        standbyRC(state);
        gLastStandbyState = eStandbyRC;
        break;
        // Other cases as needed
    }
  }
  // -----------------------LOOP ENDS--------------------------
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
  * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */ 