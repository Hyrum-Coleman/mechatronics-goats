#include <DualTB9051FTGMotorShieldUnoMega.h>
#include <Encoder.h>

DualTB9051FTGMotorShieldUnoMega myMotorDriver;
Encoder myEncoder1(21, 20);
Encoder myEncoder2(19, 18);

double GearRatio = 70;
double EncoderCountsPerRev = 64;

unsigned long t_ms = 0;
double theta1, omega1, omega1_des, omega1f = 0;
double theta1_old = 0;
double theta1_des;
double theta2, omega2, omega2_des, omega2f = 0;
double theta2_old = 0;
double theta2_des;
double V1,V2;  // motor voltage command
double t, t_old, t0, deltaT;
double Kp, Kd, Ki;  //PID gains
double error1, dErrordt1, integralError1 = 0;
double error2, dErrordt2, integralError2 = 0;
double StepSize;
double alpha;  //digital filter weight
double StepDuration;


void setup() {

  Serial.begin(250000); //need to use high baud rate when printing during feedback control
  Serial.println("Arduino Ready to Receive Control Parameters");
  myMotorDriver.init();
  myMotorDriver.enableDrivers();
  myEncoder1.write(0);
  myEncoder2.write(0);

  String inString = "";
  while (Serial.available() == 0)
    ;
  inString = Serial.readStringUntil(' ');
  Kp = inString.toFloat();
  inString = Serial.readStringUntil(' ');
  Kd = inString.toFloat();
  inString = Serial.readStringUntil(' ');
  Ki = inString.toFloat();
  inString = Serial.readStringUntil(' ');
  StepSize = inString.toFloat();
  inString = Serial.readStringUntil(' ');
  alpha = inString.toFloat();
  inString = Serial.readStringUntil(' ');
  StepDuration = inString.toFloat(); // your desired velocity for 4.5.5 is StepSize/StepDuration
  t0 = micros()/1000000.; // do this once before starting feedback control
  t_old = 0; // do this once before starting feedback control
}
void loop() {

  // Time and time step
  t = micros()/1000000.0 - t0;  // Current time minus initialization time
  deltaT = t - t_old;          // Sample time

  // Encoder sensing
  theta1 = myEncoder1.read()*2*PI/(GearRatio*EncoderCountsPerRev);  //Shaft position in radians

  // Define Desired Trajectory
  theta1_des = StepSize;  // step input = a constant desired position
  omega1_des = 0;         // desired velocity is zero for a step in desired position
 
  // Compute Errors

  error1 = theta1_des - theta1;
  omega1 = (theta1 - theta1_old) / deltaT;
  omega1f = alpha*omega1 + (1 - alpha) * omega1f;
  dErrordt1 = omega1_des - omega1f;
  integralError1 += error1 * deltaT;
  integralError1 = constrain(integralError1, -10 / Ki, 10 / Ki);
  
  // PID Control Law

  V1 = Kp * error1 + Ki * integralError1 + Kd*dErrordt1;
  
  myMotorDriver.setM1Speed(400*V1/10); // Scale and send motor command

  // Prepare for next iteration
  t_old = t;
  theta1_old = theta1;
  /////////////////////////////////////////////////////

  if (t < StepDuration) {
    Serial.print(t, 5);
    Serial.print('\t');
    Serial.print(theta1, 5);
    Serial.print('\t');
    Serial.print(dErrordt1, 5);
    Serial.print('\t');
    Serial.print(integralError1, 5);
    Serial.print('\t');
    Serial.print(V1, 5);
    Serial.print('\t');
    Serial.print(theta1_des,5);
    Serial.println();
  }
}
