#include "Sensors.h"
#include "Vector.h"

Sensors::debugMPU() {
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU sensor.");
    while (1) {
      delay(10);
    }
  }
}

Sensors::setupMPU() {
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  mpu_accel = mpu.getAccelerometerSensor();
  mpu_accel->printSensorDetails();
  mpu_gyro = mpu.getGyroSensor();
  mpu_gyro->printSensorDetails();
}

Sensors::debugBMP() {
  if (!bmp.begin_I2C()) {
    Serial.println("Failed to perform BMP sensor.");
  }
  if (!bmp.performReading()) {
    Serial.println("Failed to perform BMP reading.");
  }
}

Sensors::setupBMP() {
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
}

//Acceleration Vector
sensors_vec_t Sensors::getAcceleration() {
  mpu_accel->getEvent(&accel);
  return accel.acceleration;
}

//Angular Velocity Vector
sensors_vec_t Sensors::getGyro() {
  mpu_gyro->getEvent(&gyro);
  return gyro.gyro;
}


Sensors::getAngle(sensors_vec_t acceleration, sensors_vec_t gyro) {
  //x,y angle (IN DEGREES, without Mahony filter) 
  // float roll = 180 * atan(acceleration.y/sqrt(acceleration.x*acceleration.x+acceleration.z*acceleration.z))/ M_PI;
  // float pitch = 180 * atan(acceleration.x/sqrt(acceleration.y*acceleration.y+acceleration.z*acceleration.z))/ M_PI;

  //x,y angle (IN DEFREES, with Mahoney filter)
  mahonyFilter(gyro.x, gyro.y, gyro.z, acceleration.x, acceleration.y, acceleration.z);
  float roll = 180 * atan2(q0 * q1 + q2 * q3, 0.5 - (q1 * q1 + q2 * q2)) / M_PI;    
  float pitch = 180 * asin(2.0 * (q0 * q2 - q1 * q3)) / M_PI;
  //roll += 2.0f;
  //pitch += 0.20f;
  //Pitch is Y-axis, Roll is Y Axis
  //Serial.print("Roll Angle ");
  Serial.print(roll); Serial.print(" , ");
  //Serial.print(" Pitch Angle ");
  Serial.print(pitch);
}




float Sensors::getAltitude() {
  return bmp.readAltitude(SEALEVELPRESSURE_HPA);
}

void Sensors::mahonyFilter(float gx, float gy, float gz, float ax, float ay, float az) {
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;        

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = q1 * q3 - q0 * q2;
		halfvy = q0 * q1 + q2 * q3;
		halfvz = q0 * q0 - 0.5f + q3 * q3;
	
		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx); 
	
	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}


float Sensors::invSqrt(float x){
  float y = x;
  long i = * (long * ) &y;
  i = 0x5f3759df - (i >> 1);
  y = * (float * ) &i;
  y = y * (1.5f - ((x * 0.5f) * y * y));
  return y;
}



