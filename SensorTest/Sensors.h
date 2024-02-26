//#include <Adafruit_Sensor.h>
//#include "Adafruit_BMP3XX.h"
#include <Wire.h>
#include "math.h"

class Sensors{
  public:
    void startMPU();
    void calibrateMPU(int);
    void lowPassFilter();
    void getRotationRate(bool);
    void getAcceleration();
    void getAngle();
  private:
    float rateRoll, ratePitch, rateYaw;
    float rateCaliRoll, rateCaliPitch, rateCaliYaw;
    float accX, accY, accZ;
    float roll, pitch;
};

void Sensors::startMPU(){
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
}

void Sensors::calibrateMPU(int measurements){
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

void Sensors::lowPassFilter(){
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
}


void Sensors::getRotationRate(bool calibration){
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

void Sensors::getAcceleration(){
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
  accX = (float) accXLSB / 4096;
  accY = (float) accYLSB / 4096;
  accZ = (float) accZLSB / 4096;
}

void Sensors::getAngle(){
  roll = 180 * atan(accY / sqrt(accX * accX + accZ * accZ)) / M_PI;
  pitch = 180 * -atan(accX / sqrt(accY * accY + accZ * accZ)) / M_PI;
  Serial.print(roll); Serial.print(", ");
  Serial.println(pitch);
}
