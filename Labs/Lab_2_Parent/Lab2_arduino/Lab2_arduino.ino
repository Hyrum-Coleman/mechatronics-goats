

#include "DualTB9051FTGMotorShield.h"
DualTB9051FTGMotorShield md;

const int Apin = 0; // Analog input pin for motor voltage
const int Dpin = 6; // GDigital input pin from DAQ
const int LEDpin = 5; // Digital output pin for LED

void setup() {
  Serial.begin(9600); // Initialize serial monitor
  pinMode(Dpin,INPUT); // Set Dpin as input
  pinMode(LEDpin,OUTPUT); // Set LEDpin as output
  md.init(); // Initialize motor controller
  md.enableDrivers(); // Necessary initialization function
}

void loop() { 
  int button = digitalRead(Dpin);// Read DAQ digital state
  int Mval = analogRead(Apin); // Motor voltage 0-1023
  int M = 0;
  
  if(button) { //if true
    digitalWrite(LEDpin,HIGH); // then light the LED
    M=200;  // Initial M value. Comment out after adding the slider in the next section
    // M = map(Mval,0,1024,-400,400); // motor control value. Comment back in after adding the slider
  }
  else{ //if false
    digitalWrite(LEDpin,LOW); // then turn off LED
    M = 0; // then don't power motor
  }
  md.setM1Speed(M); // command voltage to motor
}
