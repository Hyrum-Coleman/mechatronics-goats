// Maps a goal wheel speed (returned from wheelbase kinematics) into the language of our motor driver.
// The motor driver operates on a range from -400 to 400, where negative values turn the wheel backwards.
// However, we don't neccesarily scale all the way up to -400,400. We scale up to -maxSpeed,maxSpeed, which is gDriveSpeed in our calls to this function at the moment.
void mapWheelSpeeds(float* wheelSpeeds, unsigned long maxSpeed) {
  for (int i = 0; i < cNumberOfWheels; i++) {
    wheelSpeeds[i] = map(wheelSpeeds[i], -3.91, 3.91, -1 * maxSpeed, maxSpeed);
  }
}

