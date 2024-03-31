// For PM9 only so we don't fail.
void driveInStraightLine(float distance, float driveTime) {
  Pose goalPose(0, distance, 0);
  Velocities fwdVelocities;
  Pose nextPose;

  float Kp = 1;
  // Array to store calculated errors
  float wheelSpeedsDes[cNumberOfWheels];
  // Array to store measured wheel speeds
  float odomWheelSpeeds[cNumberOfWheels];
  // Array to store control signals for the wheels
  float controlSignals[cNumberOfWheels];

  // These are analagous to omega_des in the lab9 code
  float vy_des = (goalPose.y - gRobotPose.y) / driveTime;
  int directionFactor = 1;
  if (vy_des < 0) {
    directionFactor *= -1;
  }

  double tOld = micros() / 1000000.;
  double tInit = micros() / 1000000.;
  double t, deltaT;

  float err_y, u_y;

  while ((directionFactor * gRobotPose.y) < (directionFactor * goalPose.y)) {
    t = micros() / 1000000.;
    deltaT = t - tOld;

    nextPose.y += vy_des * deltaT;

    err_y = nextPose.y - gRobotPose.y;

    u_y = Kp * err_y;

    // Transform control signals to robot coordinates
    gWheelbase->computeWheelSpeeds(0, u_y, 0, controlSignals);

    // Convert control signals from rad/sec to motor driver units
    radSecToMotorDriverSpeeds(controlSignals);

    // Set the speeds to the control signals
    gMecanumMotors.setSpeeds(controlSignals[0], -controlSignals[1], controlSignals[2], -controlSignals[3]);

    // Get current wheel speeds from encoders
    odomWheelSpeeds[0] = gWheel1Manager.getWheelSpeedRadPerSec();  // flip is taken care of in the wheel manager
    odomWheelSpeeds[1] = gWheel2Manager.getWheelSpeedRadPerSec();
    odomWheelSpeeds[2] = gWheel3Manager.getWheelSpeedRadPerSec();
    odomWheelSpeeds[3] = gWheel4Manager.getWheelSpeedRadPerSec();

    // Calculate current velocity based on wheel speeds and fwd kinematics
    // modifies fwdVelocities in place
    gWheelbase->computeVelocities(odomWheelSpeeds, fwdVelocities.xDot, fwdVelocities.yDot, fwdVelocities.thetaDot);

    // Update pose based on updated velocities
    gRobotPose.update_pos(fwdVelocities.xDot, fwdVelocities.yDot, fwdVelocities.thetaDot);

    tOld = t;
  }
  // Turn wheels back off
  gMecanumMotors.setSpeeds(0, 0, 0, 0);
  gRobotPose.reset_pose();
}

// For PM9 only so we don't fail.
void rotateInPlaceDegrees(float angle, float driveTime) {
  float angleRadians = (angle*1.07) * (M_PI / 180.0);
  Pose goalPose(0, 0, angleRadians);
  Velocities fwdVelocities;
  Pose nextPose;

  float Kp = 1;
  // Array to store calculated errors
  float wheelSpeedsDes[cNumberOfWheels];
  // Array to store measured wheel speeds
  float odomWheelSpeeds[cNumberOfWheels];
  // Array to store control signals for the wheels
  float controlSignals[cNumberOfWheels];

  // These are analagous to omega_des in the lab9 code
  float vth_des = (goalPose.theta - gRobotPose.theta) / driveTime;
  int directionFactor = 1;
  if (vth_des < 0) {
    directionFactor *= -1;
  }

  double tOld = micros() / 1000000.;
  double tInit = micros() / 1000000.;
  double t, deltaT;

  float err_th, u_th;

  while ((directionFactor * gRobotPose.theta) < (directionFactor * goalPose.theta)) {
    t = micros() / 1000000.;
    deltaT = t - tOld;

    nextPose.theta += vth_des * deltaT;

    err_th = nextPose.theta - gRobotPose.theta;

    u_th = Kp * err_th;

    // Transform control signals to robot coordinates
    gWheelbase->computeWheelSpeeds(0, 0, u_th, controlSignals);

    // Convert control signals from rad/sec to motor driver units
    radSecToMotorDriverSpeeds(controlSignals);

    // Set the speeds to the control signals
    gMecanumMotors.setSpeeds(controlSignals[0], -controlSignals[1], controlSignals[2], -controlSignals[3]);

    // Get current wheel speeds from encoders
    odomWheelSpeeds[0] = gWheel1Manager.getWheelSpeedRadPerSec();  // flip is taken care of in the wheel manager
    odomWheelSpeeds[1] = gWheel2Manager.getWheelSpeedRadPerSec();
    odomWheelSpeeds[2] = gWheel3Manager.getWheelSpeedRadPerSec();
    odomWheelSpeeds[3] = gWheel4Manager.getWheelSpeedRadPerSec();

    // Calculate current velocity based on wheel speeds and fwd kinematics
    // modifies fwdVelocities in place
    gWheelbase->computeVelocities(odomWheelSpeeds, fwdVelocities.xDot, fwdVelocities.yDot, fwdVelocities.thetaDot);

    // Update pose based on updated velocities
    gRobotPose.update_pos(fwdVelocities.xDot, fwdVelocities.yDot, fwdVelocities.thetaDot);

    tOld = t;
  }
  // Turn wheels back off
  gMecanumMotors.setSpeeds(0, 0, 0, 0);
  gRobotPose.reset_pose();
}

void driveInArcDegrees(float radius, float angleDeg, float driveTime) {
  float angleInRadians = angleDeg * (M_PI / 180.0);
  float arcLength = abs(angleInRadians * radius * 1.08);

  float odomWheelSpeeds[cNumberOfWheels];
  float controlSignals[cNumberOfWheels];

  Pose goalPose(0, arcLength, angleInRadians);
  Velocities fwdVelocities;
  Pose nextPose;

  float Kp = 1;  // Proportional gain for linear control

  // Desired velocities
  float vy_des = (goalPose.y - gRobotPose.y) / driveTime;
  float vth_des = (goalPose.theta - gRobotPose.theta) / driveTime;

  // Direction factors for linear and rotational movement
  int directionFactorLinear = vy_des < 0 ? -1 : 1;

  double tOld = micros() / 1000000.0;
  double t, deltaT;

  float err_y, u_y;    // Variables for linear control
  float err_th, u_th;  // Variables for rotational control

  while ((directionFactorLinear * gRobotPose.y) < (directionFactorLinear * goalPose.y)) {

    t = micros() / 1000000.0;
    deltaT = t - tOld;

    // Update next pose for linear and rotational components
    nextPose.y += vy_des * deltaT;
    nextPose.theta += vth_des * deltaT;

    // Calculate errors for linear and rotational movements
    err_y = nextPose.y - gRobotPose.y;
    err_th = nextPose.theta - gRobotPose.theta;

    // Calculate control signals using proportional control
    u_y = Kp * err_y;
    u_th = Kp * err_th;

    // Transform control signals to robot coordinates
    gWheelbase->computeWheelSpeeds(0, u_y, u_th, controlSignals);

    // Convert control signals from rad/sec to motor driver units
    radSecToMotorDriverSpeeds(controlSignals);

    // Set the speeds to the control signals
    gMecanumMotors.setSpeeds(controlSignals[0], -controlSignals[1], controlSignals[2], -controlSignals[3]);


    // Get current wheel speeds from encoders
    odomWheelSpeeds[0] = gWheel1Manager.getWheelSpeedRadPerSec();  // flip is taken care of in the wheel manager
    odomWheelSpeeds[1] = gWheel2Manager.getWheelSpeedRadPerSec();
    odomWheelSpeeds[2] = gWheel3Manager.getWheelSpeedRadPerSec();
    odomWheelSpeeds[3] = gWheel4Manager.getWheelSpeedRadPerSec();

    // Calculate current velocity based on wheel speeds and fwd kinematics
    // modifies fwdVelocities in place
    gWheelbase->computeVelocities(odomWheelSpeeds, fwdVelocities.xDot, fwdVelocities.yDot, fwdVelocities.thetaDot);

    // Update pose based on updated velocities
    gRobotPose.update_pos(fwdVelocities.xDot, fwdVelocities.yDot, fwdVelocities.thetaDot);

    tOld = t;
  }
  // Turn wheels back off
  gMecanumMotors.setSpeeds(0, 0, 0, 0);
  gRobotPose.reset_pose();
}


/*void driveInArcDegrees(float radius, float angle, float time) {
  float angleInRadians = angle * (M_PI / 180.0);
  float arcLength = abs(angleInRadians * radius); // always drive forwards even if turning to the right
  Pose goalPose(0, arcLength, angle); // drive forward until you rotate to the correct angle and reach the arc length
  driveToGoalPose(goalPose, time);
  gRobotPose.reset_pose();
}*/