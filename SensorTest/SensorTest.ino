#include "Sensors.h"

Sensors *sensors = new Sensors();

unsigned long currentTime = 0, previousTime = 0;
float currentAlt = 0, previousAlt;

//Wait around 10 seconds after uploading for the sensors to calibrate
void setup() {
  Serial.begin(9600);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  sensors->startMPU();
  sensors->setupBMP();
  sensors->calibrateMPU(2000);
  //sensors->calibrateBMP(2000);
  sensors->setupKalmanFilter2D();
}

void loop() {
  previousTime = currentTime;
  currentTime = micros();
  previousAlt = currentAlt;
  sensors->lowPassFilter();
  sensors->getRotationRate(true);
  sensors->getAcceleration();
  sensors->getAngle();
  sensors->getMPUVelocity();
  sensors->kalmanFilter2D();
  Serial.print(sensors->getAltitude()); Serial.print(", "); Serial.println(sensors->getVelocity());
  //Serial.println(currentTime - previousTime);
  delay(50);
}
