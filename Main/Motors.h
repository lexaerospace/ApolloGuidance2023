#include "Servo.h"

class Motors{
  public:
    calibrateMotors();
    releaseGuidance();
    float FThrust(float, float);
    float getPulseWidth(float);
    runMotor(int, int);
    runMotors(int);
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
  delay(2000);
  
}


//THRUST TO PULSE WIDTH EQUATION (g to ms): y = 25.84 * (x / 9.8 * 1000) + 893.2 
Motors::runMotor(int motor, int pulse){
  motors[motor].writeMicroseconds(pulse);
}

Motors::runMotors(int pulse){
  for(int i = 0; i < sizeof(motors); i++){
    motors[i].writeMicroseconds(pulse);
  }
  
}

//i put 90 for now put in the necessary angle to release the guidance
Motors::releaseGuidance(){
  servo.write(90);
}


//returns thrust for 1 motor in newtons
float Motors::FThrust(float v, float hi) {
  float m = 421.5 / 1000;
  float g = 9.8;
  float hf = 249.94;
  float vf = 0;
  float p = 1.225; // air density
  float A = 57.951 / 10000; // reference area
  float CDrag = 0.7028363376417; // drag coefficient
  float F = m * v*v/(2*hf - 2*hi) - m * g - (p * v*v * CDrag * A) / 2;
  float a = (v*v)/(2 * (hf - hi));
  return F/4;
}

//turns the thrust into a pulsewidth
float Motors::getPulseWidth(float thrust){

}

