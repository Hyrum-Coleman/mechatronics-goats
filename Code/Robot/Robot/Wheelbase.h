/*
 * Wheelbase.h
 * The mecanum wheel kinematics involves a set of equations for forward and inverse kinematics.
 * 
 * For forward kinematics, where we determine the robot's velocity based on wheel speeds, the equation is:
 * [w1]
 * [w2]  = (1/r) *   [ 1 -1 -(Lx + Ly)]   [vx]
 * [w3]              [ 1  1  (Lx + Ly)] * [vy]
 * [w4]              [ 1  1 -(Lx + Ly)]   [omega]
 *                   [ 1 -1  (Lx + Ly)]   
 *                                        
 * where:
 * - (vx, vy, wz) are the velocities in the robot's coordinate frame.
 * - (w1, w2, w3, w4) are the wheel speeds.
 * - Lx is the distance from the center of the robot to the wheel longitudinally.
 * - Ly is the distance from the center of the robot to the wheel laterally.
 * - r is the radius of the wheels.
 * 
 * Additionally, to solve for robot velocities using measured wheel speeds, we can use the following matrix equation:
 *          
 * [vx]    r   [ 1  1  1  1]                        [w1]
 * [vy] =  - * [-1  1  1 -1]                      * [w2]
 * [wz]    4   [-(Lx+Ly) (Lx+Ly) (Lx+Ly) -(Lx+Ly)]  [w3]
 *                                                  [w4]
*/

#ifndef WHEELBASE_H
#define WHEELBASE_H

class Wheelbase {
private:
  float Lx;  // Distance from the center of the robot to the wheel longitudinally
  float Ly;  // Distance from the center of the robot to the wheel laterally
  float r;   // Radius of the wheels

public:
  Wheelbase(float lx, float ly, float wheelRadius)
    : Lx(lx), Ly(ly), r(wheelRadius) {}

  void computeWheelSpeeds(float vx, float vy, float omega, float wheelSpeeds[4]) const {
    // Compute the factor (Lx + Ly)
    float factor = Lx + Ly;

    // Calculate wheel speeds based on the forward kinematics matrix
    /* (1/r)
    //   1  -1  -(lx+ly) 
    //   1   1   (lx+ly)
    //   1   1  -(lx+ly)
    //   1  -1   (lx+ly)
    */
    wheelSpeeds[0] = (1 / r) * (vy + vx - factor * omega);  // Wheel 1 speed, flipped vx
    wheelSpeeds[1] = (1 / r) * (vy - vx + factor * omega);  // Wheel 2 speed, flipped vx
    wheelSpeeds[2] = (1 / r) * (vy - vx - factor * omega);  // Wheel 3 speed, flipped vx
    wheelSpeeds[3] = (1 / r) * (vy + vx + factor * omega);  // Wheel 4 speed, flipped vx
  }

  // UNTESTED
  void computeVelocities(float wheelSpeeds[4], float &vx, float &vy, float &omega) const {
    // Compute the factor (Lx + Ly)
    float factor = Lx + Ly;

    // Calculate vy, vx (swapped and flipped vx), and omega based on the inverse kinematics matrix after adjustments
    /* (r/4)
    //   1          1          1          1
    //  -1          1          1         -1
    //  -1/(lx+ly)  1/(lx+ly) -1/(lx+ly)  1/(lx+ly)
    */
    vy = (r / 4) * (wheelSpeeds[0] - wheelSpeeds[1] - wheelSpeeds[2] + wheelSpeeds[3]);
    vx = (r / 4) * (wheelSpeeds[0] - wheelSpeeds[1] + wheelSpeeds[2] - wheelSpeeds[3]);
    omega = (r / (4 * factor)) * (wheelSpeeds[0] - wheelSpeeds[1] + wheelSpeeds[2] - wheelSpeeds[3]);
  }
};

#endif  // WHEELBASE_H
