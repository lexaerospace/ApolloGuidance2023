#include "Sensors.h"

Sensors::debugMPU(){
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU sensor.");
    while (1) {
      delay(10);
    }
  }
}

Sensors::setupMPU(){
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
	mpu.setGyroRange(MPU6050_RANGE_500_DEG);
	mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  mpu_accel = mpu.getAccelerometerSensor();
  mpu_accel->printSensorDetails();
  mpu_gyro = mpu.getGyroSensor();
  mpu_gyro->printSensorDetails();
}

Sensors::debugBMP(){
  if(!bmp.begin_I2C()){
    Serial.println("Failed to perform BMP sensor.");
  }
  if(!bmp.performReading()){
    Serial.println("Failed to perform BMP reading.");
  }
}

Sensors::setupBMP(){
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
}

float* Sensors::getAccel(){
  mpu_accel->getEvent(&accel);
  float accels[3] = {accel.acceleration.x, accel.acceleration.y, accel.acceleration.z};
  Serial.print("Acceleration Data: ");
  Serial.print(accel.acceleration.x);
  Serial.print(","); Serial.print(accel.acceleration.y);
  Serial.print(","); Serial.println(accel.acceleration.z);
  return accels;
}

float* Sensors::getGyro(){
  mpu_gyro->getEvent(&gyro);
  float gyros[3] = {gyro.gyro.x, gyro.gyro.y, gyro.gyro.z};
  // Serial.println("Gyro Data:");
  // Serial.print(gyro.gyro.x);
  // Serial.print(","); Serial.print(gyro.gyro.y);
  // Serial.print(","); Serial.println(gyro.gyro.z);
  return gyros;
}

float Sensors::getAltitude(){
  return bmp.readAltitude(SEALEVELPRESSURE_HPA);
}
