// Maps a goal wheel speed (returned from wheelbase kinematics) into the language of our motor driver.
// The motor driver operates on a range from -400 to 400, where negative values turn the wheel backwards.
// However, we don't neccesarily scale all the way up to -400,400. We scale up to -maxSpeed,maxSpeed, which is gDriveSpeed in our calls to this function at the moment.
void mapWheelSpeeds(float* wheelSpeeds, unsigned long maxSpeed) {
  for (int i = 0; i < cNumberOfWheels; i++) {
    wheelSpeeds[i] = map(wheelSpeeds[i], -3.91, 3.91, -1 * maxSpeed, maxSpeed);
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

  for (int i = 0; i < cNumberOfWheels; i++) {
    radSecWheelSpeeds[i] = map(radSecWheelSpeeds[i], -cRobotMaxSpeedRadSec, cRobotMaxSpeedRadSec, -cRobotDriverMaxSpeed, cRobotDriverMaxSpeed);
  }

}