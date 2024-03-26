#include "Servo.h"

class Motors{
  public:
    calibrateMotors();
    releaseGuidance();
    float FThrust(float, float);
    int getPulseWidth(float);
    runMotor(int, int);
    runMotors(int);
    testMotors();
  private:
    Servo motor1, motor2, motor3, motor4;
    Servo motors[4] = {motor1, motor2, motor3, motor4};
    Servo servo;
};

Motors::calibrateMotors(){
  motors[0].attach(5, 1000, 2000);
  motors[1].attach(6, 1000, 2000);
  motors[2].attach(9, 1000, 2000);
  motors[3].attach(10, 1000, 2000);
  servo.attach(A0);
  for(int i = 0; i < 4; i++)
    motors[i].writeMicroseconds(1000);
  delay(2000);
  
}



Motors::runMotor(int motor, int pulse){
  motors[motor].writeMicroseconds(pulse);
}

Motors::runMotors(int pulse){
  for(int i = 0; i < sizeof(motors); i++){
    motors[i].writeMicroseconds(pulse);
  }
  
}

Motors::testMotors(){
  runMotors(1200);
  delay(5000);
  runMotors(1000);
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

//WEBSITE DATA THRUST TO PULSE WIDTH EQUATION (grams to ms): pulse = (thrust + 60.329) / 0.0584
//OUR DATA THRUST TO PULSE WIDTH EQUATION (grams to ms): pulse = (thrust + 31.180) / 0.0358
int Motors::getPulseWidth(float thrust){
  //website data
  int websitePulse = (int) ((thrust + 60.329) / 0.0584);
  int experimentPulse = (int) ((thrust + 31.180) / 0.0358);
  if(expirementPulse > 2000){
    return 2000;
  } else if(experimentPulse < 1000){
    return 1000;
  }
  return experimentPulse;
}

