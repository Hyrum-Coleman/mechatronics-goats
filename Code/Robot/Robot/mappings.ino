// Maps a goal wheel speed (returned from wheelbase kinematics) into the language of our motor driver.
// The motor driver operates on a range from -400 to 400, where negative values turn the wheel backwards.
// However, we don't neccesarily scale all the way up to -400,400. We scale up to -maxSpeed,maxSpeed, which is gDriveSpeed in our calls to this function at the moment.
void mapWheelSpeeds(float* wheelSpeeds, unsigned long maxSpeed) {
  for (int i = 0; i < cNumberOfWheels; i++) {
    wheelSpeeds[i] = map(wheelSpeeds[i], -7.82, 7.82, -1 * maxSpeed, maxSpeed);
  }
}

/**
* Converts desired wheel speeds in rad sec (returned from wheelbase) to command speeds for the motors.
* 
* @param radSecWheelSpeeds : wheel speeds in rads/sec
* @return void
*/
void radSecToMotorDriverSpeeds(float* radSecWheelSpeeds) {
  // max speed of our robot is 400 or -400 motor driver units
  // max speed of our robot is _____ rad/s or _____ in/s
  // those two scales need to be correlated, and if a requested speed is greater than the max possible speed, it should be cropped

  // Find the maximum speed in the array to see if we need to scale down
  float maxSpeed = 0;
  for (int i = 0; i < cNumberOfWheels; i++) {
    if (abs(radSecWheelSpeeds[i]) > maxSpeed) {
      maxSpeed = abs(radSecWheelSpeeds[i]);
    }
  }

  // Scale down all speeds proportionally if any wheel speed exceeds the max speed in rad/s.
  // This allows us to preserve control information without just clipping to 400 on all wheels.
  if (maxSpeed > cRobotMaxSpeedRadSec) {
    float ratio = maxSpeed / cRobotMaxSpeedRadSec;
    for (int i = 0; i < cNumberOfWheels; i++) {
      radSecWheelSpeeds[i] /= ratio;
    }
  }

  // Finally, map to our motor drivers scale. Contstrain may be uneccesary now.
  for (int i = 0; i < cNumberOfWheels; i++) {
    radSecWheelSpeeds[i] = map(radSecWheelSpeeds[i], -cRobotMaxSpeedRadSec, cRobotMaxSpeedRadSec, -cRobotDriverMaxSpeed, cRobotDriverMaxSpeed);
    radSecWheelSpeeds[i] = constrain(radSecWheelSpeeds[i], -cRobotDriverMaxSpeed, cRobotDriverMaxSpeed);
  }

}

/**
* Converts an encoder count to the corresponding angular displacement of a wheel.
* 
* @param count : encoder count
* @return theta : wheel displacement in rads
*/
float encoderCountToTheta(int count) {
  // (count * 2pi) / (countsPerRev*gearRatio)
  float theta = (count * 2 * PI) / (cEncoderCountsPerRev * cWheelMotorGearRatio);
  return theta;
}

/**
* Converts an encoder count to the corresponding angular displacement of a wheel.
* 
* @param theta : angular displacement in rads
* @return inches : how far the center of the wheel would move assuming rolling without slipping
*/
float thetaToInches(float theta) {
  // x = theta*r
  float inches = theta * cWheelRadius;
  return inches;
}
