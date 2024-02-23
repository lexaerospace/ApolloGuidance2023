#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <Wire.h>
#include "math.h"

class Sensors{
  public:
    debugMPU();
    setupMPU();
    debugBMP();
    setupBMP();
    sensors_vec_t getAcceleration(); 
    sensors_vec_t getGyro();
    float* getAngle(sensors_vec_t acceleration, sensors_vec_t gyro);
    float getAltitude();
    float getVelocity(sensors_vec_t acceleration);
  private:  
    Adafruit_MPU6050 mpu;
    Adafruit_BMP3XX bmp;
    Adafruit_Sensor *mpu_accel, *mpu_gyro;
    sensors_event_t accel, gyro;
    float sampleFreq = 32.0f;
    float roll, pitch;
    float velocity = 0;
    volatile float twoKpDef = 2.0f * 5.0f;
    volatile float twoKiDef = 2.0f *0.0f;
    volatile float twoKp = twoKpDef;
    volatile float twoKi = twoKiDef;
    volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
    volatile float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f; 
    void mahonyFilter(float gx, float gy, float gz, float ax, float ay, float az);
    float invSqrt(float x);
    //The standard sea level pressure, will need to adjust for launch location
    float SEALEVELPRESSURE_HPA = 1013.25;

};