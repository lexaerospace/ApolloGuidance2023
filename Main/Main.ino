#include "Sensors.h"

Sensors* sensors = new Sensors();

void setup() {
  Serial.begin(9600);
  sensors->debugMPU();
  sensors->setupMPU();
  sensors->debugBMP();
  sensors->setupBMP();
  delay(100);
}

void loop() {
  // sensors->getAltitude();
   sensors_vec_t acceleration = sensors->getAcceleration();
   sensors_vec_t gyro = sensors->getGyro();
  // sensors_vec_t gyro = sensors->getGyro();
  // Serial.print(acceleration.x);
  // Serial.print(", "); Serial.print(acceleration.y);
  // Serial.print(", "); Serial.println(acceleration.z);
  // Serial.print(gyro.x);
  // Serial.print(", "); Serial.print(gyro.y);
  // Serial.print(", "); Serial.println(gyro.z);
  sensors->getAngle(acceleration, gyro);
  delay(100);
}
