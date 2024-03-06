#include "Servo.h"

class Motors{
  public:
    calibrateMotors();
    releaseGuidance();
    float FThrust(float, float);
  private:
    Servo motor1, motor2, motor3, motor4;
    Servo motors[4] = {motor1, motor2, motor3, motor4};
    Servo servo;
};

Motors::calibrateMotors(){
  motors[1].attach(5, 1000, 2000);
  motors[2].attach(6, 1000, 2000);
  motors[3].attach(9, 1000, 2000);
  motors[4].attach(10, 1000, 2000);
  servo.attach(A0);
  for(int i = 0; i < 4; i++)
    motors[i].writeMicroseconds(1000);
}

//i put 90 for now put in the necessary angle to release the guidance
Motors::releaseGuidance(){
  servo.write(90);
}

float FThrust(float v, float hi) {
  float m = 421.5;
  float g = 9.8;
  float hf = 249.94;
  float vf = 0;
  float p = 1.225; // air density
  float A = 57.951; // reference area
  float CDrag = 0.7028363376417; // drag coefficient
  float F = m * v*v - m * g - (p * v*v * CDrag * A) / 2;
  return F;
}