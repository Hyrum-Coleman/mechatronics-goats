#include <DualTB9051FTGMotorShieldMod3230.h>
#include <Encoder.h>

DualTB9051FTGMotorShieldMod3230 myMotorDriver;
Encoder myEncoder1(18, 22);  // wheel 1 (interrupt pin, non-interrupt pin)
Encoder myEncoder2(3, 24);   // wheel 2 (interrupt pin, non-interrupt pin)
Encoder myEncoder3(2, 26);   // wheel 3 (interrupt pin, non-interrupt pin)
Encoder myEncoder4(19, 28);  // wheel 4 (interrupt pin, non-interrupt pin)

double GearRatio = 50;
double EncoderCountsPerRev = 64;

unsigned long t_ms = 0;
double theta[4] = { 0, 0, 0, 0 }, omega[4] = { 0, 0, 0, 0 }, omega_des[4] = { 0, 0, 0, 0 }, omega_f[4] = { 0, 0, 0, 0 };
double theta_old[4] = { 0, 0, 0, 0 };
double theta_des[4];
double V[4];  // Motor voltage command
double t, t_old, t0, deltaT;
double Kp, Kd, Ki;  // PID gains
double error[4], dError_dt[4], integralError[4] = { 0, 0, 0, 0 };
double StepSize;
double alpha;  // Digital filter weight
double StepDuration;

void setup() {
  Serial.begin(250000);  // Need to use high baud rate when printing during feedback control
  Serial.println("Arduino Ready to Receive Control Parameters");
  myMotorDriver.init();
  myMotorDriver.enableDrivers();
  for (int i = 0; i < 4; ++i) {
    myEncoder1.write(0);
    myEncoder2.write(0);
    myEncoder3.write(0);
    myEncoder4.write(0);
  }

  String inString = "";
  while (Serial.available() == 0)
    ;
  Kp = Serial.parseFloat();
  Kd = Serial.parseFloat();
  Ki = Serial.parseFloat();
  StepSize = Serial.parseFloat();
  alpha = Serial.parseFloat();
  StepDuration = Serial.parseFloat();  // Your desired velocity for 4.5.5 is StepSize/StepDuration
  t0 = micros() / 1000000.;            // Do this once before starting feedback control
  t_old = 0;                           // Do this once before starting feedback control
}

void loop() {
  t = micros() / 1000000.0 - t0;  // Current time minus initialization time
  deltaT = t - t_old;             // Sample time

  // Encoder sensing for each wheel
  theta[0] = -myEncoder1.read() * 2 * PI / (GearRatio * EncoderCountsPerRev);
  theta[1] = myEncoder2.read() * 2 * PI / (GearRatio * EncoderCountsPerRev);
  theta[2] = -myEncoder3.read() * 2 * PI / (GearRatio * EncoderCountsPerRev);
  theta[3] = myEncoder4.read() * 2 * PI / (GearRatio * EncoderCountsPerRev);

  // Define Desired Trajectory for each wheel
  for (int i = 0; i < 4; ++i) {
    theta_des[i] = StepSize;  // Step input = a constant desired position
    omega_des[i] = 0;         // Desired velocity is zero for a step in desired position

    // Compute Errors for each wheel
    error[i] = theta_des[i] - theta[i];
    omega[i] = (theta[i] - theta_old[i]) / deltaT;
    omega_f[i] = alpha * omega[i] + (1 - alpha) * omega_f[i];
    dError_dt[i] = omega_des[i] - omega_f[i];
    integralError[i] += error[i] * deltaT;
    integralError[i] = constrain(integralError[i], -10. / Ki, 10. / Ki);  // Anti-windup

    // PID Control Law for each wheel
    V[i] = Kp * error[i] + Ki * integralError[i] + Kd * dError_dt[i];  // Compute voltage command
  }

  // Scale and send motor commands
  myMotorDriver.setSpeeds((400 * V[0]) / 10, -(400 * V[1]) / 10, (400 * V[2]) / 10, -(400 * V[3]) / 10);

  // Prepare for next iteration
  t_old = t;
  for (int i = 0; i < 4; ++i) {
    theta_old[i] = theta[i];
  }

  // Print statements
  if (t < StepDuration) {
    double avgTheta = 0, avgDErrorDt = 0, avgIntegralError = 0, avgV = 0, avgThetaDes = 0;

    // Compute averages
    for (int i = 0; i < 4; ++i) {
      avgTheta += theta[i];
      avgDErrorDt += dError_dt[i];
      avgIntegralError += integralError[i];
      avgV += V[i];
      avgThetaDes += theta_des[i];
    }
    avgTheta /= 4.0;
    avgDErrorDt /= 4.0;
    avgIntegralError /= 4.0;
    avgV /= 4.0;
    avgThetaDes /= 4.0;

    // Print average values
    Serial.print(t, 5);
    Serial.print('\t');
    Serial.print(avgTheta, 5);
    Serial.print('\t');
    Serial.print(avgDErrorDt, 5);
    Serial.print('\t');
    Serial.print(avgIntegralError, 5);
    Serial.print('\t');
    Serial.print(avgV, 5);
    Serial.print('\t');
    Serial.print(avgThetaDes, 5);
    Serial.println();
  }
}
