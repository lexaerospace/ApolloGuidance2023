#include "Servo.h"

class Motors{
  public:
    calibrateMotors();
    releaseGuidance();
  private:
    Servo motor1, motor2, motor3, motor4;
    Servo motors[4] = {motor1, motor2, motor3, motor4};
    Servo servo;
};

Motors::calibrateMotors(){
  motor[1].attach(5, 1000, 2000);
  motor[2].attach(6, 1000, 2000);
  motor[3].attach(9, 1000, 2000);
  motor[4].attach(10, 1000, 2000);
  servo.attach(A0);
  for(int i = 0; i < 4; i++)
    motors[i].writeMicroseconds(1000);
}

Motors::releaseGuidance(){
  servo.write(90);
}