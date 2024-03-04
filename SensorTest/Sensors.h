#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <Wire.h>
#include "math.h"
#include "BasicLinearAlgebra.h"
using namespace BLA;


class Sensors{
  public:
    startMPU();
    setupBMP();
    calibrateMPU(int);
    calibrateBMP(int);
    lowPassFilter();
    getRotationRate(bool);
    getAcceleration();
    getAngle();
    float getMPUVelocity();
    float getAltitiude();
    kalmanFilter1D(float, float, float, float);
    kalmanFilter2D();
    
  private:
    Adafruit_BMP3XX bmp;
    float rateRoll, ratePitch, rateYaw;
    float rateCaliRoll, rateCaliPitch, rateCaliYaw;
    float startAltitude;
    float accX, accY, accZ;
    float roll, pitch;
    float accZInertial;
    float mpuVelocity;
    float kalmanRoll = 0, kalmanPitch = 0; // inital predictions when program first starts
    float kalmanUncertaintyRoll = 4, kalmanUncertaintyPitch = 4;
    float kalman1DOutput[2] = {0, 0};
    float kalmanAltitude, kalmanVelocity;
    BLA::Matrix<2,2> F; BLA::Matrix<2,1> G;
    BLA::Matrix<2,2> P; BLA::Matrix<2,2> Q;
    BLA::Matrix<2,1> S; BLA::Matrix<1,2> H;
    BLA::Matrix<2,2> I; BLA::Matrix<1,1> Acc;
    BLA::Matrix<2,1> K; BLA::Matrix<1,1> R;
    BLA::Matrix<1,1> L; BLA::Matrix<1,1> M;
    float SEALEVELPRESSURE_HPA = 1013.25;
};

Sensors::startMPU(){
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
}

Sensors::setupBMP() {
  if (!bmp.begin_I2C()) {
    Serial.println("Failed to perform BMP sensor.");
  }
  if (!bmp.performReading()) {
    Serial.println("Failed to perform BMP reading.");
  }
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
}

Sensors::calibrateMPU(int measurements){
  for(int i = 0; i < measurements; i++){
    lowPassFilter();
    getRotationRate(false);
    rateCaliRoll += rateRoll;
    rateCaliPitch += ratePitch;
    rateCaliYaw += rateYaw;
    delay(1);
  }
  rateCaliRoll /= measurements;
  rateCaliPitch /= measurements;
  rateCaliYaw /= measurements;
  Serial.println("MPU Calibration Finished.");
}

Sensors::calibrateBMP(int measurements){
  for(int i = 0; i < measurements + 1; i++){
    if(i == 0){ 
      bmp.readAltitude(SEALEVELPRESSURE_HPA);
      continue;
    }
    startAltitude += bmp.readAltitude(SEALEVELPRESSURE_HPA);
    delay(1);      
  }
  startAltitude /= measurements;
  Serial.println("BMP Calibration Finished.");
}

Sensors::lowPassFilter(){
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
}


Sensors::getRotationRate(bool calibration){
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x8);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t gyroX = Wire.read() << 8 | Wire.read();
  int16_t gyroY = Wire.read() << 8 | Wire.read();
  int16_t gyroZ = Wire.read() << 8 | Wire.read();
  rateRoll = (float) gyroX / 65.5;
  ratePitch = (float) gyroY / 65.5;
  rateYaw = (float) gyroZ / 65.5;
  if(calibration){
    rateRoll -= rateCaliRoll;
    ratePitch -= rateCaliPitch;
    rateYaw -= rateCaliYaw;
    //Serial.print(rateRoll); Serial.print(", ");
    //Serial.print(ratePitch); Serial.print(", ");
    //Serial.println(rateYaw);
  }
}

Sensors::getAcceleration(){
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t accXLSB = Wire.read() << 8| Wire.read();
  int16_t accYLSB = Wire.read() << 8| Wire.read();
  int16_t accZLSB = Wire.read() << 8| Wire.read();
  accX = (float) accXLSB / 4096 - 0.01;  
  accY = (float) accYLSB / 4096 + 0.01;
  accZ = (float) accZLSB / 4096;
  //Serial.print(accX); Serial.print(", ");
  //Serial.print(accY); Serial.print(", ");
  //Serial.println(accZ);
}

Sensors::getAngle(){
  roll = 180 * atan(accY / sqrt(accX * accX + accZ * accZ)) / M_PI;
  pitch = 180 * -atan(accX / sqrt(accY * accY + accZ * accZ)) / M_PI;
  kalmanFilter1D(kalmanRoll, kalmanUncertaintyRoll, rateRoll, roll);
  kalmanRoll = kalman1DOutput[0];
  kalmanUncertaintyRoll = kalman1DOutput[1];
  kalmanFilter1D(kalmanPitch, kalmanUncertaintyPitch, ratePitch, pitch);
  kalmanPitch = kalman1DOutput[0];
  kalmanUncertaintyPitch = kalman1DOutput[1];
  roll = kalmanRoll;
  pitch = kalmanPitch - 2;
  //Serial.print(roll); Serial.print(", ");
  //Serial.println(pitch);
}

float Sensors::getMPUVelocity(){
  accZInertial = -accX * sin(pitch * M_PI / 180) + accY * sin(roll * M_PI / 180) * cos(pitch * M_PI / 180) + accZ * cos(roll * M_PI / 180) * cos(pitch * M_PI / 180);
  accZInertial = (accZInertial - 1) * 9.81 * 100; //cm/s^2
  mpuVelocity = mpuVelocity + accZInertial * 0.055;
  return mpuVelocity;
}

float Sensors::getAltitiude(){
  return (bmp.readAltitude(SEALEVELPRESSURE_HPA) - startAltitude);
}



//Right now ive set the iteration length to 0.055s set it to the time the arduino needs to do 1 loop
Sensors::kalmanFilter1D(float state, float uncertainty, float input, float measurement){
  state = state + 0.055 * input;
  uncertainty = uncertainty + 0.055 * 0.055 * 4 * 4;
  float gain = uncertainty * 1 / ( 1 * uncertainty + 3 * 3);
  state = state + gain * (measurement - state);
  uncertainty = (1 - gain) * uncertainty;
  kalman1DOutput[0] = state;
  kalman1DOutput[1] = uncertainty;
}

Sensors::kalmanFilter2D(){
  
}
