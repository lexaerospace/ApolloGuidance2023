#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <Wire.h>

class Sensors{
  public:
    debugMPU();
    setupMPU();
    debugBMP();
    setupBMP();
    
    //change float* to vectors later
    float* getAccel(); 
    float* getGyro();
    float getAltitude();

  private:  
    Adafruit_MPU6050 mpu;
    Adafruit_BMP3XX bmp;
    Adafruit_Sensor *mpu_accel, *mpu_gyro;
    sensors_event_t accel, gyro;

    //The standard sea level pressure, will need to adjust for launch location
    float SEALEVELPRESSURE_HPA = 1013.25;

};