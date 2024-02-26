#include "Sensors.h"

Sensors *sensors = new Sensors();

void setup() {
  Serial.begin(9600);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  sensors->startMPU();
  sensors->calibrateMPU(2000);
}

void loop() {
  sensors->lowPassFilter();
  sensors->getRotationRate(true);
  sensors->getAcceleration();
  sensors->getAngle();
  delay(50);
}
