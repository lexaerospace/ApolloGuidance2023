#include "Sensors.h"

Sensors *sensors = new Sensors();

unsigned long currentTime = 0, previousTime = 0;
float currentAlt = 0, previousAlt;

void setup() {
  Serial.begin(9600);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  sensors->startMPU();
  sensors->setupBMP();
  sensors->calibrateMPU(2000);
  sensors->calibrateBMP(2000);
}

void loop() {
  previousTime = currentTime;
  currentTime = micros();
  previousAlt = currentAlt;
  currentAlt = sensors->getAltitiude();
  sensors->lowPassFilter();
  sensors->getRotationRate(true);
  sensors->getAcceleration();
  sensors->getAngle();
  delay(50);

}
